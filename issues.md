# Some notes and comments on the coordinator
Filip Sandström, 2026-01-07

## Routing and scheduling algorithm
The coordinator as implemented works by detecting delays and triggering a new schedule based on the robots current position. In the original implementation, the robots are constrained to return to the starting node at the end. This means all ``task.json`` files must be rewritten to add a new job returning to the start node to achieve the same paths as before. This is however not required if not desired. 

This means that the ``routing`` function in ``E_Routing_Gurobi.py`` has been changed by removing constraint 44. An additional constraint was added to ``scheduling_model.py`` to force the start of all robots at time 0, as the robots do in fact exist at time step 0. This was a bug also in the initial implementation that would only be discovered if the a robot had a target of another robot's start position that it was due to visit prior to that other robot beginning to move.

## The coordinator class
This class contains most of the code that consitutes the Coordinator, with some small implementations to the main function in ``run_mpc.py``. 