## Title

Feedback Control For Cassie With Deep Reinforcement Learning

## Abstract

Bipedal locomotion skills are challenging to develop. **Control strategies often use local linearization of the dynamics in conjunction with reduced-order abstractions to yield tractable solutions.** In these model-based control strategies, the controller is often not fully aware of many details, including **torque limits**, **joint limits**, and other **non-linearities** that are necessarily excluded from the control computations for simplicity. Deep reinforcement learning (DRL) offers a promising **model-free** approach for controlling bipedal locomotion which can more fully exploit the dynamics. However, **current results** in the machine learning literature are often **based on ad-hoc simulation models** that are not based on corresponding hardware. Thus it remains unclear how well DRL will succeed on **realizable bipedal robots**. In this paper, we demonstrate the effectiveness of DRL using a realistic model of Cassie, a bipedal robot. By **formulating a feedback control problem as finding the optimal policy for a Markov Decision Process**, we are able to learn robust walking controllers that imitate a reference motion with DRL. Controllers for different walking speeds are learned by imitating simple timescaled versions of the original reference motion. Controller robustness is demonstrated through several challenging tests, including **sensory delay**, **walking blindly on irregular terrain** and **unexpected pushes at the pelvis**. We also show we can interpolate between individual policies and that robustness can be improved with an **interpolated policy**.

## Summary

写完笔记之后最后填，概述文章的内容，以后查阅笔记的时候先看这一段。

## Motivation

- Current results in the machine learning literature are often based on ad-hoc simulation models that are not based on corresponding hardware.

- While some work has shown the effectiveness of DRL on robots, these robots are often fully actuated. 

## Innovation

- We formulate a feedback control problem as searching for an optimal imitation policy to apply DRL to train controllers for bipedal walking tasks without needing to make the model-based simplifications.
- Controller robustness is demonstrated through several challenging tests, including sensory delay, walking blindly on irregular terrain and unexpected pushes at the pelvis. 
- We can interpolate between individual policies and that robustness can be improved with an interpolated policy.

## Method

### A. State Space and Action Space

![image-20230120121842769](https://cpy-fig-1310002510.cos.ap-chengdu.myqcloud.com/Typora_pictures/image-20230120121842769.png)

### B. Reference Motion and Simulation

- Our framework can incorporate any reference motion that describes how the robot is expected to move **over time**. 

- At the beginning of each episode, the pose of the robot is set to a state randomly selected from the reference motion.

### C. Network and Learning Algorithm

- We use an actor-critic learning framework for our ex-
  periments. (use TanH active function to limit the range of the final output.)

![image-20230120122403655](https://cpy-fig-1310002510.cos.ap-chengdu.myqcloud.com/Typora_pictures/image-20230120122403655.png)

- We use the Proximal Policy Optimization (PPO) to optimize our policy.

![image-20230120125411401](https://cpy-fig-1310002510.cos.ap-chengdu.myqcloud.com/Typora_pictures/image-20230120125411401.png)

![image-20230120125426380](https://cpy-fig-1310002510.cos.ap-chengdu.myqcloud.com/Typora_pictures/image-20230120125426380.png)

- reward function:

![image-20230120125920506](https://cpy-fig-1310002510.cos.ap-chengdu.myqcloud.com/Typora_pictures/image-20230120125920506.png)

## Evaluation

**metrics:** average rewards

**experiments:** 3D Walking; Sensory Delay; Terrain and Perturbation Test; Different Speed; Policy Interpolation

## Conclusion

- We show that our framework can produce **reasonable controllers** even **with physically infeasible reference trajectories**, such as those resulting from simple retiming of the reference motion.
- Our controllers still rely on **full state information from the robot**, while in a real world scenario the state must be **estimated from noisy sensor measurements**.
- Another future direction would be to learn a unified controller that can perform feedback control given any reasonable reference motion, to allow for zero-shot learning when given new motions for new tasks.

## Notes


