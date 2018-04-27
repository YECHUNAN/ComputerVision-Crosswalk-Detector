# ComputerVision-Crosswalk-Detector

## Overview
When taking a computer vision course, I imeplemented this crosswalk detector as the 2-week course project. The algorithms are mainly inspired by the following papers.

* [LSD: a Line Segment Detector](www.ipol.im/pub/art/2012/gjmr-lsd/article.pdf)
* [A Fast Algorithm for Finding Crosswalks using Figure-Ground Segmentation](https://pdfs.semanticscholar.org/6670/dca87158f80ababe298a481e5b6db68b5d34.pdf)

## Description
The original version is implemented in Matlab which works very well thanks to the powerful image procesing tool box provided by Matlab. I re-implemented it in OpenCV. It still works but the detection rate and the false alarm rate are both worse than the Matlab version. The algorithm mainly uses geometric features, Markov Random Field model and some hand-crafted decision rules and parameters to do the detection. It involves a lot of mathematical derivations so please refer to the papers and my report for more detail. The OpenCV version is developed using Visual Studio. You can easily open it in Visual Studio >2015 or copy all the source files and compile it on your own.

## Conclusion
The algorithm doesn't involve any learning, which means that you can use it without any prior knowledge (training data). This type of thinking is very helpful for beginners of computer vision. I hope this project can be something that inspires your thinking and new project topics.
