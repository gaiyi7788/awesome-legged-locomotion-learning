## Title

Learning Memory-Based Control for Human-Scale Bipedal Locomotion

## Abstract

Controlling a non-statically stable biped is a difficult problem largely due to the complex hybrid dynamics involved.

Recent work has demonstrated the effectiveness of reinforcement learning (RL) for simulation-based training of neural network controllers that successfully transfer to real bipeds. **The existing work, however, has primarily used simple memoryless network architectures, even though more sophisticated architectures, such as those including memory, often yield superior performance in other RL domains.** In this work, we consider **recurrent neural networks (RNNs)** for sim-to-real biped locomotion, allowing for policies that learn to use internal memory to model important physical properties. We show that while RNNs are able to significantly outperform memoryless policies in simulation, they do not exhibit superior behavior on the real biped due to overfitting to the simulation physics unless trained using dynamics randomization to prevent overfitting; this leads to consistently better sim-to-real transfer. We also show that RNNs could use their learned memory states to perform online system identification by encoding parameters of the dynamics into memory.

## Motivation

作者的写作动机

## Innovation

## Method

解决问题的方法/算法是什么？

## Evaluation

作者如何评估自己的方法，有没有问题或者可以借鉴的地方

## Conclusion

作者给了哪些strong conclusion, 又给了哪些weak conclusion?

## Notes

在这些框架外额外需要记录的笔记。
