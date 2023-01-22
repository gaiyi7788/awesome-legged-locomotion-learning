## Title

Learning Memory-Based Control for Human-Scale Bipedal Locomotion

## Abstract

Controlling a non-statically stable biped is a difficult problem largely due to the complex hybrid dynamics involved.

Recent work has demonstrated the effectiveness of reinforcement learning (RL) for simulation-based training of neural network controllers that successfully transfer to real bipeds. **The existing work, however, has primarily used simple memoryless network architectures, even though more sophisticated architectures, such as those including memory, often yield superior performance in other RL domains.** In this work, we consider **recurrent neural networks (RNNs)** for sim-to-real biped locomotion, allowing for policies that learn to use internal memory to model important physical properties. We show that while **RNNs are able to significantly outperform** memoryless policies in **simulation**, they do **not exhibit superior behavior on the real biped** due to overfitting to the simulation physics unless trained using `dynamics randomization to prevent overfitting`; this leads to consistently better sim-to-real transfer. We also show that RNNs could use their learned memory states to perform online system identification by encoding parameters of the dynamics into memory.

## Motivation

- Memory-based controllers, such as recurrent neural networks (RNN), are a potentially powerful choice for solving highly dynamic nonlinear control problems.
- The introducing of RNN increases the potential for overfitting to the simulation physics.

## Innovation

- we consider recurrent neural networks (RNNs) for sim-to-real biped locomotion, allowing for policies that learn to use internal memory to model important physical properties.
- We show that RNNs trained using dynamics randomization could prevent overfitting and leads to consistently better sim-to-real transfer.
- We also show that RNNs could use their learned memory states to perform online system identification by encoding parameters of the dynamics into memory.

## Method

![image-20230122093754790](https://cpy-fig-1310002510.cos.ap-chengdu.myqcloud.com/Typora_pictures/image-20230122093754790.png)

![image-20230122102308141](https://cpy-fig-1310002510.cos.ap-chengdu.myqcloud.com/Typora_pictures/image-20230122102308141.png)

**A. State Space and Action Space**

![image-20230122102347013](https://cpy-fig-1310002510.cos.ap-chengdu.myqcloud.com/Typora_pictures/image-20230122102347013.png)

**B. Reward Design**

![image-20230122101121372](https://cpy-fig-1310002510.cos.ap-chengdu.myqcloud.com/Typora_pictures/image-20230122101121372.png)

**C. Dynamics Randomization**

At the start of each episode during training, we **randomize 61 dynamics parameters.** 

We aggressively **randomize the pelvis center of mass**, and we find that it **introduces enough noise** into the simulation dynamics to overcome the sim-to-real gap.

![image-20230122101335498](https://cpy-fig-1310002510.cos.ap-chengdu.myqcloud.com/Typora_pictures/image-20230122101335498.png)

**D. Recurrent Proximal Policy Optimization**

- Instead of sampling individual timesteps from the replay buffer, we sample batches of entire trajectories for PPO training.
- In addition, we collect statistics including the mean state $s_µ$ and standard deviation $s_σ$ before training, and use these to normalize states during training and during evaluation. We found this prenormalization step to be very important for consistent performance.

## Evaluation

- We trained ten LSTM networks and ten FF networks with dynamics randomization, and ten LSTM networks and ten FF networks without dynamics randomization, each with separate random seeds.
- When **trained without dynamics randomization**, **LSTM networks** attain a significantly **higher reward** than feedforward networks, with surprisingly little variance. We attribute this difference to the **LSTM overfitting** the dynamics of the simulation.

![image-20230122105529786](https://cpy-fig-1310002510.cos.ap-chengdu.myqcloud.com/Typora_pictures/image-20230122105529786.png)

- We conducted a robustness test in simulation across ten chosen sets of dynamics. 
- We were unable to get FF policies trained with dynamics randomization to work on hardware.
- We note that all of the LSTM policies trained with dynamics randomization are able to successfully walk on Cassie.

![image-20230122105720175](https://cpy-fig-1310002510.cos.ap-chengdu.myqcloud.com/Typora_pictures/image-20230122105720175.png)

- We assume that in order for a memory-based agent to succeed in an environment with unknown dynamics, it is advantageous to maintain a compressed history of states that includes encoded estimates about those dynamics to determine which action to take.

## Conclusion

## Notes
