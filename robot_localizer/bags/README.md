We used different maps to test our particle filter and collected data from the runs which we have stored in the .bag files. 

Our files: 

1. **ac109_1_testing** - This map is larger and has less features. We needed to adjust the radius of the circle around the robot to a smaller value account for this. Since there aren't many distinct features and more area, it was important to scale down the radius variable to prevent false positives.

<a href='https://youtu.be/IlxChRHc4kA'> <button type="button" name="button" style="font-family: inherit;
    font-size: 100%;
    padding: 0.5em 1em;
    color: rgba(0, 0, 0, 0.80);
    border: none rgba(0, 0, 0, 0);
    background-color: #E6E6E6;
    text-decoration: none;
    border-radius: 2px;" class="btn">Watch Video</button></a>



2. **ac109_2_testing**- This map is smaller and has more unique features, like an obstacle. This tests the robustness of the likelihood model we implemented since the object is not modeled in the map. 

<a href='https://youtu.be/gIMAhhX1jnE'><button type="button" name="button" style="font-family: inherit;
    font-size: 100%;
    padding: 0.5em 1em;
    color: rgba(0, 0, 0, 0.80);
    border: none rgba(0, 0, 0, 0);
    background-color: #E6E6E6;
    text-decoration: none;
    border-radius: 2px;" class="btn">Watch Video</button></a>