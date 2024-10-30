# K23a

## Requirements

```sh
> sudo apt install build-essential
```

## Compilation

To compile the program just run the following:

```sh
> make
```

## Examples

```sh
> ./bin/k23a -c ./conf.yaml -d ./.sample/siftsmall/siftsmall_base.fvecs -q ./.sample/siftsmall/siftsmall_query.fvecs --verbose
```

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

## Sample Data

The sample data used for this project can be found in [corpus-texmex.irisa.fr](http://corpus-texmex.irisa.fr/).
It consists of large-scale datasets of high-dimensional feature vectors extracted from real-world images.

## Contributors

- [Χρήστος Καλτσάς 1115202000289](https://github.com/xcalts)
- [Γιδάκος Βασίλης 1115200800022](https://github.com/sdi0800022)
- [Κρικελλη Ναταλια 1115202000104](https://github.com/nataliakrik)