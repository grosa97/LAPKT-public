
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

#ifndef __Q_NOVELTY_HEURISTIC__
#define __Q_NOVELTY_HEURISTIC__

#include <search_prob.hxx>
#include <heuristic.hxx>
#include <ext_math.hxx>
#include <strips_state.hxx>
#include <strips_prob.hxx>
#include <vector>
#include <deque>
#include <climits>
#include <algorithm>

namespace aptk
{

	namespace agnostic
	{

		template <typename Search_Model, typename Node>
		class Q_Novelty_Heuristic : public Heuristic<State>
		{
		public:
            typedef typename Search_Model::State_Type State;

			Q_Novelty_Heuristic(const Search_Model &prob, const unsigned max_MB = 2048)
					: Heuristic<State>(prob), m_strips_model(prob.task()), m_max_memory_size_MB(max_MB), m_verbose(true), m_rp_fl_only(false)
			{
				init();
                // m_base_h = base_h;
                set_arity(1);
                m_V = m_strips_model.num_fluents();
			}

			void set_verbose(bool v) { m_verbose = v; }

			void set_rp_fl_only(bool v) { m_rp_fl_only = v; }

			virtual ~Q_Novelty_Heuristic()
			{
			}

			void init()
			{
				// typedef typename std::vector<Search_Node *>::iterator Node_Ptr_It;

				// for (Node_Ptr_It it = m_nodes_tuples.begin(); it != m_nodes_tuples.end(); it++)
				// 	*it = NULL;
                
                std::fill(m_fact_scores.begin(), m_fact_scores.end(), 0);
			}

			unsigned arity() const { return m_arity; }

			unsigned set_arity(unsigned max_arity, unsigned partition_size = 0)
			{
                // /*currently only supports arity of 1!!*/
                // assert(max_arity=1);

				m_arity = max_arity;
				m_num_tuples = 1;
				m_num_fluents = m_strips_model.num_fluents();

				float size_novelty = ((float)pow(m_num_fluents, m_arity) / 1024000.) * sizeof(Node *);
				if (m_verbose)
					std::cout << "Try allocate size: " << size_novelty << " MB" << std::endl;
				if (size_novelty > m_max_memory_size_MB)
				{
					m_arity = 1;

					size_novelty = ((float)pow(m_num_fluents, m_arity) / 1024000.) * sizeof(Node *);
					if (m_verbose)
						std::cout << "EXCEDED, m_arity downgraded to 1 --> size: " << size_novelty << " MB" << std::endl;
				}

				for (unsigned k = 0; k < m_arity; k++)
					m_num_tuples *= m_num_fluents;

				// m_nodes_tuples.resize(m_num_tuples, NULL);
                m_fact_scores.resize(m_num_tuples, UINT_MAX);
				return m_arity;
			}

			// void eval(Node *n, float &h_val)
			// {
			// 	compute(n, h_val);
			// }

			// void eval(Node *n, float &h_val, std::vector<Action_Idx> &pref_ops)
			// {
			// 	eval(n, h_val);
			// }

            // void eval(Node *n, float &h_val) {
            //     compute_count_metric(n, h_val);
            //     update_counts(n);
            // }

			virtual void eval(Node *n, int base_val, int &h_val)
			{
                compute(n, base_val, h_val);
			}

			virtual void eval(const State &s, float &h_val)
			{
				assert(true);
			}

			virtual void eval(const State &s, float &h_val, std::vector<Action_Idx> &pref_ops)
			{
				assert(true);
			}

		protected:

        bool n_has_state(Node *n){
            return n->state() != NULL;
        }

        void compute( Node *n, int h_val, int &h_qb_val)
        {
            const bool has_state = n_has_state(n);

            if (!has_state)
                n->parent()->state()->progress_lazy_state(m_strips_model.actions()[n->action()]);

            // const Fluent_Vec &fl = s.fluent_vec();
            const Fluent_Vec &fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();
            
            h_qb_val = calc_h_qb(fl, h_val);

            if (!has_state)
                n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()]);
        }

        int calc_h_qb(const Fluent_Vec &fl, int h_val)
        {
            // std::unordered_set<unsigned> seen;
            
            int n_minus = 0;
            int n_plus = 0;

            for (auto f : fl)
            {
                // if (seen.find(f) == seen.end()) 
                {
                    // seen.insert(f);
                    if ( h_val > m_fact_scores[f] )
                        n_minus++;
                    else if ( h_val < m_fact_scores[f] )
                    {
                        n_plus++;
                        m_fact_scores[f] = h_val;
                    }
                }
            }

            if (n_plus > 0)
                return m_V - n_plus;
            else
                return m_V + n_minus;
        }


			const STRIPS_Problem &m_strips_model;
			// std::vector<Search_Node *> m_nodes_tuples;
            std::vector<int> m_fact_scores;
			unsigned m_arity;
			unsigned long m_num_tuples;
			int m_num_fluents;
			unsigned m_max_memory_size_MB;
			bool m_verbose;
			bool m_rp_fl_only;
            int m_V;
            // Base_Heuristic& m_base_h;
		};

	}

}

#endif // q_novelty_heuristic.hxx
