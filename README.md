# Robot Programming ROS2 
These are the code to solve the challenges [in this repository](https://mygit.th-deg.de/gaydos/tb3-maze-challenges).

# Contributors
I have developed all the code alone. Hence I am the only contributer here. 

### ***Details:***  
***Name:*** Jimmy Tan   
***Matriculation number:*** 00819296 

# Contents 
- <mark>reports</mark> contains all my report files, written in Markdown.
- <mark>media</mark> contains all the screenshots and GIFs required in the reports.
- <mark>src</mark> contains all the python codes and world files for the simulations. 

# Usage 
Install the Python Library <mark>mako</mark>. This project uses the <mark>mako-render</mark> tool. 

    pip install mako

Check if this works: 

    which mako-render 

Clone this repository and run the world files and models. Suppose you want to run the world file <mark>world_5_5.sdf</mark>

    cd src/worlds
    ./play world_5_5.sdf 

In another terminal, run the python code for the corresponding challenges. Suppose you want to run the code for <mark>challenge 5</mark>:

    cd src 
    python3 challenge5.py 

# Link to report

For the report, it is deployed on the internet and you can check it here: [Jimmy Tan's report for this project](https://jimmytan2000.github.io/thd-robot-programming-report-webserver/).