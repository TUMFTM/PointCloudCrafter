---
title: 'PointCloudCrafter: Tools for Handling, Manipulating and Analyzing of Point Clouds'
tags:
  - point cloud
  - ROS2
authors:
  - name: Dominik Kulmer
    orcid: 0000-0000-0000-0000
    equal-contrib: true
    affiliation: 1 # (Multiple affiliations must be quoted)
  - name: Maximilian Leitenstern
    orcid: 0009-0008-6436-7967
    equal-contrib: true # (This is how you can denote equal contributions between multiple authors)
    affiliation: 1
affiliations:
 - name: Institute of Automotive Technology, Department of Mobility System Engineering, School of Engineering and Design, Technical University of Munich, Germany
   index: 1
date: 19.November 2025  
bibliography: paper.bib

# Optional fields if submitting to a AAS journal too, see this blog post:
# https://blog.joss.theoj.org/2018/12/a-new-collaboration-with-aas-publishing
# aas-doi: 10.3847/xxxxx  [Add to Citavi project by DOI]  <- update this with the DOI from AAS once you know it.
# aas-journal: Astrophysical Journal <- The name of the AAS journal.
---

# Summary

A toolkit for extracting, manipulating, and evaluating point clouds and 3D spatial maps. Includes functions for processing, analyzing, and visualizing point clouds, designed to streamline workflows in 3D mapping and general point cloud handling. Ideal for researchers and developers working with LiDAR, SLAM, and 3D spatial data.

# Statement of need



# Tools and Functionalities

PointClouCrafter comprises the following tools and functionalities:
- ROS2 tools
  - pointcloudcrafter
  This package provides two executables to handle `sensor_msgs::msg::PointCloud2` within rosbags:
    1. Transform and/or concatenate single pointclouds based on their timestamp and extract them to `.pcd` files.
  - tum_pcd_modifyer using PCL [@Rusu2011]
  - tum_rosbag_analyzer
  A package to process rosbags and analyze various values. Currently, the following modes are implemented:
    1. timestamps: Analyze the header timestamp and the individual points timestamps of pointcloud2 messages.

# Citations

<!-- Citations to entries in paper.bib should be in
[rMarkdown](http://rmarkdown.rstudio.com/authoring_bibliographies_and_citations.html)
format. -->

If you want to cite a software repository URL (e.g. something on GitHub without a preferred
citation) then you can do it with the example BibTeX entry below for @fidgit.

For a quick reference, the following citation commands can be used:
- `@author:2001`  ->  "Author et al. (2001)"
- `[@author:2001]` -> "(Author et al., 2001)"
- `[@author1:2001; @author2:2001]` -> "(Author1 et al., 2001; Author2 et al., 2002)"

# Figures

Figures can be included like this:
![Caption for example figure.\label{fig:example}](figure.png)
and referenced from text using \autoref{fig:example}.

Figure sizes can be customized by adding an optional second parameter:
![Caption for example figure.](figure.png){ width=20% }

# Acknowledgements

We acknowledge contributions from several students that contributed to the single tools
during their student research project:
- Tools: Performance-Studi

# References
