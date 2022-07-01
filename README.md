<h1>Cao_OJCSYS2022</h1>

This code supplements the OJ-CSYS 2022 Special Section submission "Efficient Learning of Hyperrectangular Invariant Sets using Gaussian Processes" by Michael E. Cao, Matthieu Bloch, and Samuel Coogan.

<h2>Abstract</h2>
We present a method for efficiently computing reachable sets and forward invariant sets for continuous-time systems with dynamics that include unknown components. Our main assumption is that, given any hyperrectangle of states, lower and upper bounds for the unknown components are available. With this assumption, the theory of mixed monotone systems allows us to formulate an efficient method for computing a hyperrectangular set that over-approximates the reachable set of the system. We then show a related approach that leads to sufficient conditions for identifying hyperrectangular sets that are forward invariant for the dynamics. We additionally show that set estimates tighten as the bounds on the unknown behavior tighten. Finally, we derive a method for satisfying our main assumption by modeling the unknown components as state-dependent Gaussian processes, providing bounds that are correct with high probability. A key benefit of our approach is to enable tractable computations for systems up to moderately high dimension that are subject to low dimensional uncertainty modeled as Gaussian processes, a class of systems that often appears in practice. We demonstrate our results on several examples, including a case study of a planar multirotor aerial vehicle.

<h2>Notes on Repository</h2>
The scripts contained within this repository can be used to reproduce the results from the paper.

For reference:

* The Autonomous Vehicle case study can be produced by running `bicycle_decomp_tight.m`
* The forward invariant set example can be produced by running `functionSolver.m`
* The Planar Multirotor case study can be produced by running `pvtol_casestudy.m`



