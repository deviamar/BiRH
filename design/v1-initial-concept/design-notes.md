# Initial Concept Design Notes

## Core Design Philosophy
My first prototype focused on achieving **force closure** with a simple geometric structure.

![Initial test](BiRH/visuals/images/Initial_Test.png)

## Key Design Decisions

**Three-Finger Configuration**
- Three fingers to achieve force closure, even as the hand is rotated about the palm
- Arranged symmetrically around the palm to reduce spacing between fingers to maximize object coverage
- Identical finger design for interchangeability and rotational symmetry

**Human-Inspired Phalanges**
- Three phalanges per finger for multiple contact points
- Phalange sizes scaled from my own forefinger measurements
- Length decreases from proximal to distal: secure the object base first, then refine grip for stability

**Simple Actuation System**
- Holes in the phalanges CAD model for string routing
- One motor per finger via single string for underactuated control with spring opposition
- Focus of the prototype was on testing the effectiveness of the geometry, not refined tendon mechanics

**Wrist Rotation**
- Palm designed to rotate about the wrist joint to test grasp stability when grasping objects

## Early Insights
Initially thought this design would only work for sphere-like or soft objects, but testing showed it worked better than expected across various shapes. The main challenge was achieving proper closure sequence as I wanted proximal phalanges to close first for full object enclosure. Could approximate this by tightening the screws to different degrees to control friction. 

## Next Steps (at the time)
Planned to design a second prototype with better implementation of spring and string such that pulling the string will enable the finger to close while releasing it will allow the finger to extend due to spring. Planned to explore underactuated mechanisms and tendon routing strategies for sequential phalange closure.

## Further explorations and thoughts
This simple design raised several questions I wanted to explore:

- How would an additional degree of freedom, rotating each finger about the palm perpendicular to the current hinge joint, improve grasping ability?
- What about testing different finger placement positions around the palm?
- Could a deformable palm material help with grasping stability?
- How would a more complex thumb mechanism enable improved grasping?
- How to design a thumb motion mechanism that more authentically captures the complex motion of the human thumb?
