---
title: 'PointCloudCrafter: Tools for Handling, Manipulating and Analyzing of Point Clouds'
tags:
  - point cloud
  - C++
  - ROS2
  - PCL
authors:
  - name: Dominik Kulmer
    orcid: 0000-0001-7886-7550
    equal-contrib: true
    affiliation: 1
  - name: Maximilian Leitenstern
    orcid: 0009-0008-6436-7967
    equal-contrib: true
    affiliation: 1
affiliations:
 - name: Institute of Automotive Technology, Department of Mobility System Engineering, School of Engineering and Design, Technical University of Munich, Germany
   index: 1
date: 25.January 2026  
bibliography: paper.bib

---

# Summary

PointCloudCrafter is a C++ command-line interface (CLI) toolkit for the processing, manipulation, and analysis of three-dimensional point cloud data. The software provides a collection of compiled executables for integration into automated research pipelines, enabling common point cloud operations without requiring custom code.

The toolkit supports conversion and processing of data from multiple acquisition and storage formats, including ROS2 bag recordings, binary files [@nuScenes2019; @KITTI2012], and standard PLY and PCD files. A central design feature of PointCloudCrafter is its schema-agnostic handling of per-point attributes. Arbitrary scalar fields associated with individual points are preserved throughout conversion, transformation, filtering, and aggregation steps, ensuring that auxiliary metadata remains available for downstream analysis.

By combining support for robotic middleware data with batch-oriented file processing, PointCloudCrafter facilitates reproducible workflows across a range of scientific and engineering applications that rely on large-scale three-dimensional point cloud data.

# Statement of need

Three-dimensional point cloud data is widely used in robotics, autonomous driving, remote sensing, and scientific data analysis. In practice, such data is commonly encountered in two distinct representations. During acquisition and experimentation, point clouds are often recorded as time-stamped data streams using robotic middleware such as ROS2. For benchmarking, archival, and offline analysis, the same data is typically stored and distributed as static files in dataset-specific or standardized formats.

Transitioning between these representations, or performing large-scale batch operations on point cloud collections, remains a recurring challenge. The Point Cloud Library (PCL) [@Rusu2011] provides a comprehensive set of data structures and algorithms for three-dimensional processing, but its use generally requires direct integration into custom software projects. Graphical tools such as CloudCompare are well-suited for interactive inspection but are not designed for automated, headless processing of large datasets. Python-based libraries such as Open3D [@Zhou2018] and Pyoints [@Lamprecht2019] offer accessible interfaces for point cloud manipulation, but performance and input-output overhead can become limiting factors when processing large directories of high-resolution scans.

A further limitation of many existing tools is the implicit assumption of fixed-point schemas. Conversion utilities and parsers often restrict data to spatial coordinates and optional color information, discarding non-standard scalar fields. However, many scientific applications depend on additional per-point attributes. Automotive LiDAR sensors provide channels such as intensity, timestamp, and ring index, which are required for calibration, motion correction, and mapping. In other domains, including structural biology, point clouds may encode physicochemical properties such as hydrophobicity or electrostatic potential [@Parigger2024]. Preserving these attributes across processing steps is essential for reproducibility and downstream analysis.

PointCloudCrafter addresses these needs by providing a lightweight CLI toolkit focused on batch-oriented point cloud conversion and processing. The primary software contribution is a schema-agnostic, command-line-driven framework that unifies middleware-based and file-based point cloud processing while preserving arbitrary per-point attributes. Arbitrary scalar fields refer to dynamically discovered per-point attributes whose names, data types, and memory layout are preserved without enforcing a predefined schema.

# Functionality

PointCloudCrafter provides two primary execution modes, *rosbag* and *file*, for streaming and static data workflows, respectively.

The *rosbag* mode enables extraction of `sensor_msgs::msg::PointCloud2` topics from ROS2 bag recordings. This mode integrates with the `TF2` library to perform time-resolved pose lookups based on the recorded transform tree. Each point cloud can be transformed from the sensor coordinate frame into a user-specified fixed reference frame using the pose corresponding to the exact acquisition timestamp. This enables motion-consistent export of point cloud data from dynamic sensor platforms.

The *file* mode focuses on batch processing of static point cloud files. Supported formats include common text and binary formats such as PLY, PCD, OBJ, and TXT, as well as dataset-specific binary formats used in KITTI and nuScenes. In this mode, users can apply rigid-body transformations, merge multiple point clouds into a single representation, and perform filtering operations. Implemented filters include geometric cropping using box, sphere, or cylindrical regions, voxel grid downsampling, and statistical or radius-based outlier removal.

Both execution modes rely internally on the `PCLPointCloud2` data structure, enabling schema-agnostic handling of point attributes. All processing steps are fully parameterized via command-line options, enabling deterministic, reproducible execution across datasets.

A typical use case is the extraction of LiDAR scans from ROS2 bag recordings, transformation into a global reference frame using an externally estimated trajectory, and aggregation into a static map for offline evaluation. This workflow can be executed in a single batch process while preserving sensor-specific scalar fields required for subsequent analysis.

# Acknowledgements

The authors acknowledge the developers and maintainers of the Point Cloud Library (PCL) [@Rusu2011], upon which PointCloudCrafter is built. We also acknowledge the contributions of student researchers at the Institute of Automotive Technology at the Technical University of Munich who supported the development and evaluation of this software.

# References
