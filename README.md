# team-11
Team 11 MASLAB 2024 Code

![](https://github.com/alexrice236/MASLAB2024-team11/blob/main/im/logo.png)

## Robot Restrictions
  20” (L) x 20” (W) x 30” (H) starting bounding box
  
  Cannot be built to intentionally cause harm to other robots
 
  Cannot touch the floor of the opposing side
  
  You can touch the walls, though.
  
  No flying please!
  
  Don’t treat field walls as stable structural elements (because they aren’t built to be)
  
## Game & Field
  **How long is a game?**
  
  3 minutes (may change) per side. A round is made of two games. A team plays once on one side of the field, and then switches to the other.

  **How big is the field?**
  
  Not predefined, but it will be ~15 ft in diameter, with some interior walls that need to be navigated.
  
  **What shape is the field?**
  
  Not predefined. It may change between rounds of the competition. The only given constraint is that internal wall angles will be greater than or equal to 90 degrees.
  
  **Size of the center platform?**
  
  The height is 4 in. It is 8 in deep (dimension perpendicular to the wall) and will be some multiple of a foot long. There will be one center platform with straddling the two sides.
  
  **How many cubes?**
  
  Both sides start with five stacks of three cubes of alternating colors. If you are the red team then you will have stacks of green-red-green. If you are the green team you will have stacks of red-green-red.

## Scoring
### Scoring Method 1 : Stack Cubes

  height^2 points for free standing stack of cubes of your own color on your side.
  
  height^3 points for free standing stack of cubes of opposing color on the central platform.

### Scoring Method 2 : Hold Cubes

  +1 point for each cube (any color) remaining in possession of your robot after it is removed from the field.

# The RAM (Robotic Autonomous Manipulator)

Collects and sorts blocks according to color, holds opposition blocks, stacks friendly blocks.

![](https://github.com/alexrice236/MASLAB2024-team11/blob/main/im/maslab0.gif)

![](https://github.com/alexrice236/MASLAB2024-team11/blob/main/im/maslab1.gif)

![](https://github.com/alexrice236/MASLAB2024-team11/blob/main/im/maslab2.gif)

![](https://github.com/alexrice236/MASLAB2024-team11/blob/main/im/maslab4.gif)



