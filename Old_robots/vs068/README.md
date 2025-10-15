# Veggie Arm: DENSO VS-068

## Overview

The **DENSO VS-068** is a compact, high-performance 6-axis industrial robot widely deployed in high-speed assembly, handling, and collaborative automation.  
It offers an excellent balance of **reach**, **agility**, and **safety**, making it ideal for small to medium-sized workspaces such as our sandwich-making environment [Denso][1][4][5].

## Key Specifications

| Feature       | Specification             |
|---------------|---------------------------|
| **Axes**      | 6                         |
| **Max Reach** | 710 mm                    |
| **Payload**   | 7 kg                      |
| **Repeatability** | ±0.02 mm              |
| **Weight**    | 49 kg                     |
| **Cycle Time**| 0.33 sec (1 kg, 300 mm move) |
| **Motion Range** | J1: ±170°, J2: +135/-100°, J3: +153/-120°, J4: ±270°, J5: ±120°, J6: ±360° |
| **Mounting**  | Floor, Wall, or Ceiling   |
| **Safety**    | Collision detection & collaborative-ready [Denso Safety][10] |

## Why the VS-068 Was Chosen

- **Agility & Coverage**  
  Its **710 mm reach** and **0.33-second cycle time** make it well-suited for handling toppings (lettuce, tomato, cheese) spread across multiple bins. This fits the project’s requirement for **fast, flexible coverage** of the workspace [Denso Specs][2].

- **Payload & Precision**  
  A **7 kg payload** and **±0.02 mm repeatability** ensure it exceeds the minimal needs for lightweight food items, while providing the repeatable accuracy required in assembly tasks [RoboDK][3].

- **Safety & Collaboration**  
  The VS-068 includes **collision detection functions** and supports programming for **active collision avoidance**. These align perfectly with the assignment’s emphasis on multi-robot safety and collaborative operation [Denso Safety][10].

- **Assignment Compliance**  
  This robot:  
  - Is **not** in Peter Corke’s Robotics Toolbox,  
  - Is **not** a Universal Robot or UTS-supplied arm,  
  - Has openly available **documentation and CAD**,  
  making it fully compliant with the project’s rules [Assignment PDF][11].

## Role in the Assembly Line (“Veggie Arm”)

- **Task**: Pick and place fresh toppings with speed and precision.  
- **Justification**: Prioritises **workspace coverage** and **dexterity**, contrasting with heavier payload arms (e.g., “meat” arm) or extended-reach delivery arms.  
- **Integration**: Operates in shared workspace with other robots, relying on **software-based collision avoidance** and **barrier models** to ensure safe, flexible collaboration.

---

## References

1. [DENSO Robotics VS-068 Product Page][1]  
2. [VS-068 / VS-087 Technical Datasheet (PDF)][2]  
3. [RoboDK VS-068 Model][3]  
4. [DENSO Robotics Overview][4]  
5. [DENSO Manuals and Specs][5]  
6. [DENSO VSA Series Overview][6]  
7. [VS-068/087 Datasheet (Scribd)][7]  
8. [Used Machines: Denso VS-068][8]  
9. [DENSO Robotics Global Page][9]  
10. [Collision Detection Function Overview][10]  
11. [UTS Assignment Brief – Lab Assignment 2 (PDF)][11]  

[1]: https://www.denso-wave.com/en/robot/product/five-six/vs068-087.html  
[2]: https://www.densorobotics-europe.com/fileadmin/Products/VS_-_068_and_VS_087-Technical_Data_Sheet/VS_-_068_and_VS_087-Technical_Data_Sheet.pdf  
[3]: https://robodk.com/robot/Denso/VS-068  
[4]: https://www.densorobotics-europe.com/product-overview/products/5-and-6-axis-robots/vs-068-087/  
[5]: http://eidtech.dyndns-at-work.com/support/rc8_manual/000020.html  
[6]: https://www.densorobotics.com/products/5-6-axis/vsa-series/  
[7]: https://www.scribd.com/document/666813623/DENSO-Robotics-Datasheet-vs-068-087-Series-1  
[8]: https://www.used-machines.com/denso-vs-068/gm-920-0318  
[9]: https://www.denso-wave.com/en/robot/  
[10]: https://www.denso-wave.com/en/robot/product/function/Collision-detection-function.html  
[11]: Lab-Assignment-2-Pick-and-Place-DoGoodBot-1-1.pdf  
