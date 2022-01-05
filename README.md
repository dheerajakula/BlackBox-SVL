## Installing the Package
- From Github 
    1. Clone the repository:
    `git clone https://github.com/dheerajakula/BlackBox-SVL`
    2. Install the necessary packages
        ```
        cd BlackBox-SVL
        poetry install
        ```

## Installing SVL simulator
1. Clone the repository make sure you have git lfs installed.
`git clone https://github.com/dheerajakula/simulator`
2. Change the current branch.
`git checkout release-2021.1`
3. Open the simulator with Unity Editor Version 2019.4.18f1 and create a python API only simulation.

## Running a falsification example
1. By following a one dimensional signal from psytaliro
`poetry run python examples/FollowInputSignal.py`
2. By following a two dimensional signal from psytaliro
`poetry run python examples/FollowInputSignal2D.py`
3. By following points from psytaliro
`poetry run python examples/FollowPoints.py`