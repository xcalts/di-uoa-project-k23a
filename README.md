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
    Implementations of the Vamana and Filtered Vamana indexing.
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
    <li><a href="#requirements">Requirements</a></li>
    <li><a href="#compilation">Compilation</a></li>
    <li><a href="#getting-started">Getting Started</a></li>
    <li><a href="#folder-structure">Folder Structure</a></li>
    <li><a href="#datasets">Datasets</a></li>
    <li><a href="#contributors">Contributors</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
  </ol>
</details>



## Requirements

```sh
> sudo apt install build-essential
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Compilation

To compile the program just run the following:

```sh
> make debug
> make release  
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Getting Started

```sh
> make release
> ./bin/vamana --conf ./conf.yaml
> ./bin/vamana-filtered --conf ./conf.yaml
> ./bin/vamana-stiched --conf ./conf.yaml
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Folder Structure

Here is the folder structure explained:

```text
.
├── .docs/   # Documentation
├── .sample/ # Samples
├── inc/     # Headers
├── libs/    # Libraries
├── src/     # Source
└── tests/   # Unit-Tests
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Datasets

### Vamana

The sample data used for this project can be found in [corpus-texmex.irisa.fr](http://corpus-texmex.irisa.fr/).
It consists of large-scale datasets of high-dimensional feature vectors extracted from real-world images.

### Filtered Vamana

Our datasets, both released and evaluation set, are derived from the YFCC100M Dataset. Each dataset comprises vectors encoded from images using the CLIP model, which are then reduced to 100 dimensions using Principal Component Analysis (PCA). Additionally, categorical and timestamp attributes are selected from the metadata of the images. The categorical attribute is discretized into integers starting from 0, and the timestamp attribute is normalized into floats between 0 and 1.

For each query, a query type is randomly selected from four possible types, denoted by the numbers 0 to 3. Then, we randomly choose two data points from dataset D, utilizing their categorical attribute (C) timestamp attribute (T), and vectors, to determine the values of the query. Specifically:

1. Randomly sample two data points from D.
2. Use the categorical value of the first data point as *v* for the equality predicate over the categorical attribute C.
3. Use the timestamp attribute values of the two sampled data points for the range predicate. Designate *l* as the smaller timestamp value and *r* as the larger. The range predicate is thus defined as *l ≤ T ≤ r*.
4. Use the vector of the first data point as the query vector.
5. If the query type does not involve *v*, *l*, or *r*, their values are set to -1.

We assure that at least 100 data points in D meet the query limit.

| # | Name | Description | Dataset Size | Query set Size |
|---|-------|-------------|--------------|----------------|
| 1 | dummy-data.bin / dummy-queries.bin | dummy data and queries for packing submission in reprozip | 10^4 | 10^2 |
| 2 | contest-data-release-1m.bin / contest-queries-release-1m.bin | medium scale released data and queries | 10^6 | 10^4 |
| 3 | contest-data-release-10m.bin / contest-queries-release-10m.bin | large-scale released data and queries | 10^7 | 4 * 10^6 |
| 5 | secret-data-10m.bin / secret-queries-10m.bin | secret large-scale data and queries, used for evaluation | 10^7 | 4 * 10^6 |

The dummy dataset is included in [transactionalblog/sigmod-contest-2024 GitHub](#). For the contest datasets, please download them from [Zenodo](#).


<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Contributors

- [Χρήστος Καλτσάς 1115202000289](https://github.com/xcalts)
- [Ναταλια Κρικελλη 1115202000104](https://github.com/nataliakrik)

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
