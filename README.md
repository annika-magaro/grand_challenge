# LPARA*
Lifelong Planning Anytime Repairing A* (LPARA*) is a novel algorithm that combines the existing Lifelong Planning A* (LPA*) and Anytime Repairing A* (ARA*) algorithms to accommodate both **changing environments** and **constraints on the amount of time available for path planning**. The algorithm weights heuristics to rapidly path plan and decrements the weight as time permits to replan a more optimal path (as done in the ARA* algorithm), but it works in real time such that if a change in the environment is detected (i.e., the addition or removal of an obstacle), planning is briefly stopped to make all inconsistent nodes in the previous search tree (identified by calculating RHS values like in the LPA* algorithm) and their neighbors consistent again before resuming the path planning process at the last value of _e_, the weight of the heuristic.  

## Algorithm Details
The following is the pseudocode of our algorithm, which integrates the LPA* and ARA* algorithms.
```
procedure Initialize()
    set g(s) to infinity for all s
    insert all s in OPEN with key fvalue(s)

procedure fvalue(s)
    return g(s) + e * h(s)

procedure UpdateVertex(s):
    if s in OPEN: remove s from OPEN
    if s is s_start then
        rhs = 0
    else
        rhs = min(g(s') + c(s', s) for s' in predecessors(s))
    
    if g(s) != rhs: insert s in OPEN with key fvalue(s)

procedure ImprovePath()
    while fvalue(s_goal) > min(fvalue(s) for s in OPEN):
        check for changes in environment
        for each changed node s
            UpdateVertex(s)

        remove s with smallest fvalue(s) from OPEN
        add s to CLOSED
        for each successor s' of s
            if s' was not visited before then
                g(s') = infinity
            if g(s') > g(s) + c(s, s') then
                g(s') = g(s) + c(s, s')
                if s' not in CLOSED then
                    insert s' in OPEN with fvalue(s')
                else
                    insert s' in INCONS

procedure Searching()
    Initialize()
    ImprovePath()
    publish current e'-suboptimal solution

    e' = min(e, s_goal / min(g(s) + h(s) for s in OPEN U self.INCONS))
    while e' > 1
        decrement e 
        move nodes from INCONS to OPEN
        update the priorities of all nodes s in OPEN according to fvalue(s)
        clear CLOSED
        ImprovePath()
        publish current e'-suboptimal solution


```
## Implementation
We created a robust implementation that can cover a wide variety of applications and to enable us to exhaustively test the benefits of LPARA* by creating many configurable parameters, which are listed below. We utilize `pyplot` from `matplotlib` to provide a graphical user interface to showcase the gridworld with the obstacles, the planned paths, and the visited nodes over time. The user can press on the gridworld to add or remove obstacles and the planned path will dynamically update. 

The obstacles in the gridworld are randomly generated based on the specified coverage amount.  

| Flag        | Description                                                                                                                                                                               | Default |
|-------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| -e          | _float >= 1_ : Starting weight of heuristic. A higher value will result in faster planning but an increasingly suboptimal solution.                                                       | 2.5     |
| -connected  | _"4" or "8"_ : Connectivity of the gridworld. A 4-connected world supports up, down, left, and right movements. An 8-connected world supports movements along all four diagonals as well. | 4       |
| -size       | _int > 45_ : Number of rows and columns in gridworld.                                                                                                                                     | 50      |
| -clump-size | _"small", "medium", or "large"_ Size of obstacles that are dynamically placed by user.                                                                                                    | medium  |
| -coverage   | _float >= 0 and <= 0.5_ : Percentage of gridworld covered by an obstacle.                                                                                                                 | 0.1     |     

## Evaluation
In order to evaluate performance of LPARA*, we created an automated version of LPARA*. This does not support user clicks but instead changes the obstacles in the grid by some amount after each call to ImprovePath(). Evaluation introduces two new flags.

| Flag        | Description                                                                                                                                                                               | Default |
|-------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| -percent-change          | _float >= 0 and <= 0.5_  : Percent of obstacles to remove/add each round. Higher values will lead to a more dynamic gridworld.                                                       | 0.1     |
| -plot  | _True or False_ : Whether to include plots | True       |

## Usage
A demonstration of the LPARA* algorithm can be run using the following setup:
```
python3 ./lpara_star_realtime.py
```

The parameters can be configured as follows, in any order. Any parameter that is not specified uses its default value specified above.
```
python3 ./lpara_star_realtime.py -connected 8 -size 50 -clump-size  large -coverage 0.2
```
A window should pop up with the demonstration. Obstacles can be added by clicking on the grid.

If a user wishes to use the automated setup, they can run the following (example parameters provided):
```
python3 ./lpara_star_validation.py -connected 4 -size 30 -clump-size medium -coverage 0.05 -percent-change 0.1 -plot True
```
A window will pop up with a demonstration if plot is set to True. After it is done, it will return the number of states expanded.

To run evaluation, users can run the validate_all() function in lpara_star_validation.py.

## Demo
* [LPARA* Demo with Small Obstacles](https://youtu.be/-YL-jlHuD-k)
* [LPARA* Demo with Large Obstacles](https://youtu.be/hmiuN_hZIZk)

* [LPA* Validation Test Example](https://youtu.be/xlyuO17h2rM)
* [ARA* Validation Test Example](https://youtu.be/A1dI_yH-PQI)
* [LPARA* Validation Test Example](https://youtu.be/GEfqqgrk-Fo)




