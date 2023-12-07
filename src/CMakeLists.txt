
#----- Project Source -----#

add_subdirectory(wrapper)

# Translate PDDL input (leverages external libraries, including tarski)
add_subdirectory(translate)

# Search engines 
add_subdirectory(engine)

# Algorithms to evaluate a node
add_subdirectory(node_eval)

# Blueprints for strips model
add_subdirectory(model)

# lapkt template library(ltl) a set of utility blueprints
add_subdirectory(utils)

# Planners which leverage engine(s) in some particular configuration
add_subdirectory(planner)

# Python packaging related
add_subdirectory(python)