
/*
Lightweight Automated Planning Toolkit

Copyright 2022
Miquel Ramirez <miquel.ramirez@unimelb.edu.au>Nir Lipovetzky <nirlipo@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files
(the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge,
publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject
 to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __COUNT_NOVELTY_SA_PARTITION__
#define __COUNT_NOVELTY_SA_PARTITION__

#include <search_prob.hxx>
#include <heuristic.hxx>
#include <ext_math.hxx>
#include <strips_state.hxx>
#include <strips_prob.hxx>
#include <vector>
#include <deque>

namespace aptk
{

	namespace agnostic
	{

		template <typename Search_Model, typename Search_Node>
		class Count_Novelty_SA_Partition : public Heuristic<State>
		{
		public:
			Count_Novelty_SA_Partition(const Search_Model &prob, unsigned max_arity = 1, const unsigned max_MB = 2048)
					: Heuristic<State>(prob), m_strips_model(prob.task()), m_max_memory_size_MB(max_MB), m_always_full_state(false), m_partition_size(0), m_verbose(true), m_rp_fl_only(false)
			{
				set_arity(max_arity, 1);
				init();
			}

			void set_verbose(bool v) { m_verbose = v; }

			void set_rp_fl_only(bool v) { m_rp_fl_only = v; }

			virtual ~Count_Novelty_SA_Partition()
			{
			}

			void init()
			{
				typedef typename std::vector<std::vector<Search_Node *>>::iterator Node_Vec_Ptr_It;
				typedef typename std::vector<Search_Node *>::iterator Node_Ptr_It;
				typedef typename std::vector<std::vector<int>>::iterator Int_Vec_Ptr_It;

				for (Node_Vec_Ptr_It it_p = m_nodes_tuples_by_partition.begin(); it_p != m_nodes_tuples_by_partition.end(); it_p++)
					for (Node_Ptr_It it = it_p->begin(); it != it_p->end(); it++)
						*it = NULL;

				for (Int_Vec_Ptr_It it_p = m_tuple_counts_by_partition.begin(); it_p != m_tuple_counts_by_partition.end(); it_p++)
					std::fill(it_p->begin(), it_p->end(), 0);
			}

			unsigned arity() const { return m_arity; }
			void set_full_state_computation(bool b) { m_always_full_state = b; }
			Search_Node *table(unsigned partition, unsigned idx) { return m_nodes_tuples_by_partition[partition][idx]; }
			unsigned &partition_size() { return m_partition_size; }
			bool is_partition_empty(unsigned partition) { return m_nodes_tuples_by_partition[partition].empty(); }

			void set_arity(unsigned max_arity, unsigned partition_size = 0)
			{
				assert(max_arity == 1);
				m_partition_size = partition_size;
				m_arity = max_arity;
				m_num_tuples = 1;
				m_num_fluents = m_strips_model.num_fluents();
				m_num_actions = m_strips_model.num_actions();
				m_num_tuples = m_num_fluents * (m_num_actions + 1);

				float size_novelty = ((float)pow(m_num_tuples, 1) / 1024000.) * (float)partition_size * sizeof(Search_Node *);
				// std::cout << "Try allocate size: "<< size_novelty<<" MB"<<std::endl;
				if (size_novelty > m_max_memory_size_MB)
				{
					m_arity = 1;
					size_novelty = ((float)pow(m_num_tuples, 1) / 1024000.) * (float)partition_size * sizeof(Search_Node *);

					std::cout << "EXCEDED, m_arity downgraded to 1 --> size: " << size_novelty << " MB" << std::endl;
				}

				// for (unsigned k = 0; k < m_arity; k++)
				// 	m_num_tuples *= m_num_fluents;
				m_num_tuples = m_num_fluents * (m_num_actions + 1);

				m_nodes_tuples_by_partition.resize(partition_size + 1);
				m_tuple_counts_by_partition.resize(partition_size + 1);

				for (unsigned i = 0; i < partition_size + 1; i++)
				{
					m_nodes_tuples_by_partition[i].clear();
					m_tuple_counts_by_partition[i].resize(m_num_tuples, 0);
				}
				
			}

			virtual void eval(Search_Node *n, float &h_val)
			{
				compute_count_metric_sa_1(n, h_val);				
				update_counts_sa(n);
			}

			virtual void eval(const State &s, float &h_val)
			{
				assert(true);
			}

			virtual void eval(const State &s, float &h_val, std::vector<Action_Idx> &pref_ops)
			{
				assert(true);
			}

            void update_counts_sa(Search_Node *n) {
                float redundant_variable = 0;
                // compute(n, redundant_variable);
				compute(n, redundant_variable); //testing
            }

			void eval_no_update(Search_Node *n, float &h_val) {
				compute_count_metric_sa(n, h_val);
			}

		protected:

			void check_table_size(Search_Node *n)
			{

				if (m_partition_size < n->partition())
				{
					m_nodes_tuples_by_partition.resize(n->partition() + 1);
					m_tuple_counts_by_partition.resize(n->partition() + 1);
					m_partition_size = n->partition();
				}

				if (m_nodes_tuples_by_partition[n->partition()].empty())
					m_nodes_tuples_by_partition[n->partition()].resize(m_num_tuples, NULL);

				if (m_tuple_counts_by_partition[n->partition()].empty())
					m_tuple_counts_by_partition[n->partition()].resize(m_num_tuples, 0);
			}
			/**
			 * If can use add(op), the computation is F^i-1 aprox. FASTER!!!
			 * if action == no_op (i.e. start action), the computation is F^i, SLOWER!!
			 * where i ranges over 1 to max_arity
			 */


			void compute(Search_Node *n, float &novelty)
			{

				novelty = (float)m_arity + 1;
				
				if (n->partition() == std::numeric_limits<unsigned>::max())
					return;

				check_table_size(n);

				for (unsigned i = 1; i <= m_arity; i++)
				{

#ifdef DEBUG
					if (m_verbose)
						std::cout << "search node: " << n << std::endl;
#endif
					
                    // bool new_covers = n->action() == no_op ? cover_tuples(n, i) : cover_tuples_op(n, i);
					/**
					 * Ensure only using cover_tuples for incrementing counts because need to increment also fluents already present
					 * in state and not on added list.
					*/
					// bool new_covers = cover_tuples(n, i);
					// bool new_covers = cover_tuples_op(n, i);
					bool new_covers =  cover_tuples_sa(n, i);

#ifdef DEBUG
					if (m_verbose && !new_covers)
						std::cout << "\t \t PRUNE! search node: " << n << std::endl;
#endif
					if (new_covers)
						if (i < novelty)
							novelty = i;
				}
			}

			bool n_has_state(Search_Node *n){
				return n->state() != NULL;
			}

			std::vector<unsigned> get_fa_vec(Search_Node* n, Fluent_Vec& fluents)
			{
				std::vector<unsigned> fa_vec;
				unsigned a_i = n->action() != -1 ? n->action() : m_num_actions;

				for (auto f_i: fluents)
				{
					fa_vec.push_back( a_i*m_num_fluents + f_i );
				}
				return fa_vec;
			}

			bool cover_tuples_sa(Search_Node *n, unsigned arity)
			{

				//arity only = 1 for this implementation
				assert(arity == 1);
				// const bool has_state = n->has_state();
				const bool has_state = n_has_state(n);

				// if (!has_state)
				// 	n->parent()->state()->progress_lazy_state(m_strips_model.actions()[n->action()]);
				static Fluent_Vec added, deleted, temp_fv;
				if (!has_state)
				{
					
					added.clear();
					deleted.clear();
					temp_fv.clear();
					temp_fv.assign(n->parent()->state()->fluent_vec().begin(), n->parent()->state()->fluent_vec().end());
					n->parent()->state()->progress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
				}

				Fluent_Vec &fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();
				std::vector<unsigned> fa_vec = get_fa_vec(n, fl);
				// /*debug*/
				// std::cout << fl <<std::endl;
				// /*----*/

				bool new_covers = false;

				std::vector<unsigned> tuple(arity);

				unsigned n_combinations = aptk::unrolled_pow(fa_vec.size(), arity);

#ifdef DEBUG
				if (m_verbose)
					std::cout << n << " covers: " << std::endl;
#endif

				for (unsigned idx = 0; idx < n_combinations; idx++)
				{
					/**
					 * get tuples from indexes
					 */
					idx2tuple(tuple, fa_vec, idx, arity); /*gets a tuple for checking novelty, using idx to determine the respective fluents in fl to create the tuple, & arity for tuple size*/

					/**
					 * Check if tuple is covered
					 */
					unsigned tuple_idx;

					if (arity == 1)
					{
						tuple_idx = tuple2idx(tuple, arity);
					}
					// else if (arity == 2)
					// {
					// 	if (tuple[0] == tuple[1])
					// 		continue; // don't check singleton tuples
					// 	tuple_idx = tuple2idx_size2(tuple, arity);
					// }
					else
					{

						// If all elements in the tuple are equal, ignore the tuple
						if (std::any_of(tuple.cbegin(), tuple.cend(), [&tuple](unsigned x)
														{ return x != tuple[0]; }))
							continue;
						tuple_idx = tuple2idx(tuple, arity);
					}

					/**
					 * new_tuple if
					 * -> none was registered
					 * OR
					 * -> n better than old_n
					 */

					auto &n_seen = m_nodes_tuples_by_partition[n->partition()][tuple_idx];
                    
					/*increment tuple counts for partition*/
					m_tuple_counts_by_partition[n->partition()][tuple_idx]++;

					if (!n_seen || is_better(n_seen, n))
					{
						n_seen = (Search_Node *)n;
						new_covers = true;
					}
				}
				if (!has_state)
				{
					n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
					n->parent()->state()->fluent_vec().assign(temp_fv.begin(), temp_fv.end());
				}
				// if (!has_state)
				// 	n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()]);
				return new_covers;
			}

			// specialized version for tuples of size 2
			inline unsigned tuple2idx_size2(std::vector<unsigned> &indexes, unsigned arity) const
			{
				unsigned min = indexes[0] <= indexes[1] ? indexes[0] : indexes[1];
				unsigned max = indexes[0] <= indexes[1] ? indexes[1] : indexes[0];
				return min + max * m_num_fluents;
			}

			// general version for tuples of arbitrary size
			inline unsigned tuple2idx(std::vector<unsigned> &indexes, unsigned arity) const
			{
				unsigned idx = 0;
				unsigned dimension = 1;

				for (int i = arity - 1; i >= 0; i--)
				{
					idx += indexes[i] * dimension;
					dimension *= m_num_fluents;
				}

				return idx;
			}

			inline void idx2tuple(std::vector<unsigned> &tuple, Fluent_Vec &fl, unsigned idx, unsigned arity) const
			{
				unsigned next_idx, div;
				unsigned current_idx = idx;
				unsigned n_atoms = fl.size();

				for (unsigned i = arity - 1; i >= 0; i--)
				{
					div = aptk::unrolled_pow(n_atoms, i);

					if (current_idx < div)
					{
						next_idx = current_idx;
						current_idx = 0;
					}
					else
					{
						next_idx = current_idx % div;
						// if current_idx is zero and is the last index, then take next_idx
						current_idx = (current_idx / div != 0 || i != 0) ? current_idx / div : next_idx;
					}

					tuple[i] = fl[current_idx];

					current_idx = next_idx;
					if (i == 0)
						break;
				}
			}

			void compute_count_metric_sa_1(Search_Node *n, float &metric_value) 
			{

				if (n->partition() == std::numeric_limits<unsigned>::max())
					return;

				check_table_size(n);

                unsigned arity = 1;
		
                metric_value = 0;

				const bool has_state = n_has_state(n);

                // const bool has_state = n->has_state();

                // if (!has_state)
				// 	n->parent()->state()->progress_lazy_state(m_strips_model.actions()[n->action()]);
				static Fluent_Vec added, deleted, temp_fv;
				if (!has_state)
				{
					added.clear();
					deleted.clear();
					temp_fv.clear();
					temp_fv.assign(n->parent()->state()->fluent_vec().begin(), n->parent()->state()->fluent_vec().end());
					n->parent()->state()->progress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
				}

                Fluent_Vec &fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();
				std::vector<unsigned> fa_vec = get_fa_vec(n, fl);

                std::vector<unsigned> tuple(arity);

                unsigned n_combinations = aptk::unrolled_pow(fa_vec.size(), arity);

                for (unsigned idx = 0; idx < n_combinations; idx++)
                {
                    /**
					 * get tuples from indexes
					 */
					idx2tuple(tuple, fa_vec, idx, arity); /*gets a tuple for checking novelty, using idx to determine the respective fluents in fl to create the tuple, & arity for tuple size*/

					/**
					 * Check if tuple is covered
					 */
					unsigned tuple_idx;
					unsigned tuple_count;

                    /*if arity = 1*/
                    tuple_idx = tuple2idx(tuple, arity);

                    tuple_count = m_tuple_counts_by_partition[n->partition()][tuple_idx];

					// float debug_val = (float)1 / (1 + tuple_count); //DEBUG
                    /*subtract to get negative of novelty metric, such that lower value means greater surprise*/
                    metric_value -= (float)1 / (1 + tuple_count);
                }
				if (!has_state)
				{
					n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
					n->parent()->state()->fluent_vec().assign(temp_fv.begin(), temp_fv.end());
				}
                // if (!has_state)
				//     n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()]);
			}

			// inline bool is_better(Search_Node *n, Search_Node *new_n) const
			// {
			// 	return (new_n->gn() < n->gn());
			// 	return (new_n->gn_unit() < n->gn_unit());
			// 	// return false;
			// }
			inline bool is_better(Search_Node *n, const Search_Node *new_n) const
			{
				// return false;
				return new_n->is_better(n);
			}

			Fluent_Set* get_rp_set(Search_Node *n) 
			{
				Search_Node* n_start = n;
				while (!n_start->rp_vec())
				{
					n_start = n_start->parent();
				}	
				return n_start->rp_set();
			}

			const STRIPS_Problem &m_strips_model;
			std::vector<std::vector<Search_Node *>> m_nodes_tuples_by_partition;
			std::vector<std::vector<int>> m_tuple_counts_by_partition;
			unsigned m_arity;
			unsigned long m_num_tuples;
			unsigned m_num_fluents;
			unsigned m_num_actions;
			unsigned m_max_memory_size_MB;
			bool m_always_full_state;
			unsigned m_partition_size;
			bool m_verbose;
			bool m_rp_fl_only;
		};

	}

}

#endif // count_novelty_sa_partition.hxx
