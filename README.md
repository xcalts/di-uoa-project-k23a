<!-- Improved compatibility of back to top link: See: https://github.com/xcalts/di-uoa-project-k23a/pull/73 -->

<a id="readme-top"></a>

[![C++][C++]][C++-url]
[![Contributors][contributors-shield]][contributors-url]
[![Tests][tests-shield]][tests-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/xcalts/di-uoa-project-k23a">
    <img src=".github/logo.png" alt="Logo" width="80" height="80">
  </a>

  <h3 align="center">Project K23a</h3>

  <p align="center">
    Implementations of the plain, filtered and stiched vamana indexing algorithms.
    <br />
    <a href="https://github.com/xcalts/di-uoa-project-k23a/issues/new?labels=bug&template=bug-report---.md">Report Bug</a>
    ·
    <a href="https://github.com/xcalts/di-uoa-project-k23a/issues/new?labels=enhancement&template=feature-request---.md">Request Feature</a>
  </p>

</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#showcase">Showcase</a></li>
    <li><a href="#team">Team</a></li>
    <li><a href="#requirements">Requirements</a></li>
    <li><a href="#build">Build</a></li>
    <li><a href="#getting-started">Getting Started</a></li>
    <li><a href="#folder-structure">Folder Structure</a></li>
    <li><a href="#libraries">Libraries</a></li>
    <li><a href="#used-datasets">Used Datasets</a></li>
    <li><a href="#system">System</a></li>
    <li><a href="#optimizations">Optimizations</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#references">References</a></li>
  </ol>
</details>

## Showcase

Here is an example of how it works:

<div align="center">
  <a href="https://github.com/xcalts/di-uoa-project-k23a">
    <img src=".github/showcase.gif" alt="Logo" width="800">
  </a>
</div>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Team

<div align="center">

  <p>National Kapodistrian University of Athens - Departments of Informatics</p>

| #   | Name                                               | Email                | UUID          |
| --- | -------------------------------------------------- | -------------------- | ------------- |
| 1   | [Christos Kaltsas](https://github.com/xcalts)      | sdi2000289@di.uoa.gr | 1115202000289 |
| 2   | [Natalia Krikelli](https://github.com/nataliakrik) | sdi2000104@di.uoa.gr | 1115202000104 |

</div>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Requirements

To build the project, you will need to install the following packages.

```sh
> sudo apt install build-essential
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Build

To build the project, run the following:

```sh
> make clean    # Remove previous generated artifacts.
> make release  # Build optimized artifacts.
> make debug    # Build debuggable artifacts.
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Tests

To execute the unit tests, run the following:

```sh
> ./bin/unitests
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Getting Started

To run the program, execute the following commands:

```sh
> # Perform the "Vamana" indexing algorithm.
> ./bin/app initialize --conf ./conf.yaml --algo "vamana"
> # Perform the "Filtered Vamana" indexing algorithm.
> ./bin/app initialize --conf ./conf.yaml --algo "filtered-vamana"
> # Perform the "StichedVamana" indexing algorithm.
> ./bin/app initialize --conf ./conf.yaml --algo "stiched-vamana"
> # Evaluate the "Vamana" indexing algorithm
> ./bin/app evaluate --conf ./conf.yaml --algo "vamana"
> # Evaluate the "Filtered Vamana" indexing algorithm
> ./bin/app evaluate --conf ./conf.yaml --algo "filtered-vamana"
> # Evaluate the "Stiched Vamana" indexing algorithm
> ./bin/app evaluate --conf ./conf.yaml --algo "stiched-vamana"
> # Generate the groundtruth neighbors of the queries
> ./bin/app generate-groundtruth --conf ./conf.yaml
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Folder Structure

Here is the folder structure explained:

```sh
├── assets/     # Text visualizations of the datasets.
├── data/       # Dataset used for the algorithms.
├── docs/       # Documentation (papers) that document the algorithms.
├── inc/        # Headers.
├── libs/       # Header-Only libraries.
├── src/        # Source code.
├── store/      # The place that algorithms are stored after the initialization.
├── tests/      # Unit tests.
├── README.md   # Project documentation.
├── conf.yaml   # Parameters for the program.
├── LICENSE.txt # Copyrights.
└── Makefile    # Build commands.
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Libraries

The program uses the following external libraries:

| Name                                                | Description                                    |
| --------------------------------------------------- | ---------------------------------------------- |
| [acutest](https://github.com/mity/acutest)          | Simple header-only C/C++ unit testing facility |
| [argh](https://github.com/adishavit/argh)           | A minimalist argument handler                  |
| [rapidyaml](https://github.com/biojppm/rapidyaml)   | Parse and emit YAML, and do it fast            |
| [indicators](https://github.com/p-ranav/indicators) | Activity Indicators for Modern C++             |
| [termcolor](https://github.com/termcolor/termcolor) | ANSI color formatting for output in terminal   |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Used Datasets

### TEXMEX Dataset - Evaluation of Plain Vamana

It consists of large-scale datasets of high-dimensional feature vectors extracted from real-world images.

| Filename                     | Size   | Dimensions |
| ---------------------------- | ------ | ---------- |
| shiftsmall_base.fvecs        | 10.000 | 128        |
| shiftsmall_query.fvecs       | 100    | 128        |
| shiftsmall_groundtruth.ivecs | 100    | 100        |

- [TEXMEX - Official](http://corpus-texmex.irisa.fr/)

### Sigmod Contest Dataset - Evaluation of Filterd|Stiched Vamana

Both released and evaluation set, are derived from the YFCC100M Dataset.

Each dataset comprises vectors encoded from images using the CLIP model, which are then reduced to 100 dimensions using Principal Component Analysis (PCA). Additionally, categorical and timestamp attributes are selected from the metadata of the images.

The categorical attribute is discretized into integers starting from 0, and the timestamp attribute is normalized into floats between 0 and 1.

For each query, a query type is randomly selected from four possible types, denoted by the numbers 0 to 3.

Then, we randomly choose two data points from dataset D, utilizing their categorical attribute (C) timestamp attribute (T), and vectors, to determine the values of the query. Specifically:

1. Randomly sample two data points from D.
2. Use the categorical value of the first data point as _v_ for the equality predicate over the categorical attribute C.
3. Use the timestamp attribute values of the two sampled data points for the range predicate. Designate _l_ as the smaller timestamp value and _r_ as the larger. The range predicate is thus defined as _l ≤ T ≤ r_.
4. Use the vector of the first data point as the query vector.
5. If the query type does not involve _v_, _l_, or _r_, their values are set to -1.

We assure that at least 100 data points in D meet the query limit.

| Filename          | Size   | Dimensions | Additional Parameters |
| ----------------- | ------ | ---------- | --------------------- |
| dummy-data.bin    | 10,000 | 100        | 2                     |
| dummy-queries.bin | 1,000  | 100        | 4                     |

> [!NOTE]
> Our implementations do not take advantage of the timestamp attributes.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## System

All the times were recorded on the following system:

```sh
Linux Distribution:            Debian GNU/Linux 12 (bookworm)
Linux Kernel:                  6.1.0-28-amd64
Computer Model:                Microsoft Corporation Virtual Machine Hyper-V UEFI Release v4.1
Processor (CPU):               13th Gen Intel(R) Core(TM) i7-13800H
CPU Sockets/Cores/Threads:     1/4/8
CPU Caches:                    L1d: 48 KiB × 4 (192KiB)
                               L1i: 32 KiB × 4 (128KiB)
                               L2: 1,280 KiB × 4 (5.0MiB)
                               L3: 24,576 KiB × 1 (24MiB)
Architecture:                  x86_64 (64-bit)
Total memory (RAM):            7,939 MiB (7.8GiB) (8,324 MB (8.4GB))
Total swap space:              975 MiB (1,023 MB)
Disk space:                    sda: 40,960 MiB (40GiB) (42,949 MB (43GB))
Computer name:                 debian
Hostname:                      debian
IPv4 addresses:                eth0: [REDACTED]
                               br-3bf608224bfc: [REDACTED]
                               docker0: [REDACTED]
                               br-dbe1e0b54d58: [REDACTED]
MAC addresses:                 eth0: [REDACTED]
                               br-3bf608224bfc: [REDACTED]
                               docker0: [REDACTED]
                               br-dbe1e0b54d58: [REDACTED]
Computer ID:                   6d3e852c44a242c68065962e08cc7e2e
Time zone:                     US/Eastern (EST, -0500)
Language:                      en_US.UTF-8
Virtual Machine hypervisor:    microsoft
Bash Version:                  5.2.15(1)-release
Terminal:                      xterm-256color

```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Optimizations

Our team implemented the following optimizations:

| Description                                         | Before | After | Change (%) |
| --------------------------------------------------- | ------ | ----- | ---------- |
| Parallelization of medoid calculation using threads | 40 sec | 7 sec | 82.5%      |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## References

- [DiskANN: Fast Accurate Billion-point Nearest Neighbor Search on a Single Node](https://suhasjs.github.io/files/diskann_neurips19.pdf)
- [Filtered-DiskANN: Graph Algorithms for Approximate Nearest Neighbor Search with Filters](https://dl.acm.org/doi/10.1145/3543507.3583552)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->

[contributors-shield]: https://img.shields.io/github/contributors/xcalts/di-uoa-project-k23a.svg?style=for-the-badge
[contributors-url]: https://github.com/xcalts/di-uoa-project-k23a/graphs/contributors
[tests-shield]: https://img.shields.io/github/actions/workflow/status/xcalts/di-uoa-project-k23a/ci.yml?style=for-the-badge&label=TESTS
[tests-url]: https://github.com/xcalts/di-uoa-project-k23a/actions/workflows/ci.yml
[stars-shield]: https://img.shields.io/github/stars/xcalts/di-uoa-project-k23a.svg?style=for-the-badge
[stars-url]: https://github.com/xcalts/di-uoa-project-k23a/stargazers
[issues-shield]: https://img.shields.io/github/issues/xcalts/di-uoa-project-k23a.svg?style=for-the-badge
[issues-url]: https://github.com/xcalts/di-uoa-project-k23a/issues
[license-shield]: https://img.shields.io/github/license/xcalts/di-uoa-project-k23a.svg?style=for-the-badge
[license-url]: https://github.com/xcalts/di-uoa-project-k23a/blob/master/LICENSE.txt
[C++]: https://img.shields.io/badge/-C++-blue?style=for-the-badge
[C++-url]: https://cplusplus.com/
