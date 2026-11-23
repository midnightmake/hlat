# HabSat Log Analysis Tool

A command-line utility for generating summaries of mission log files.

Please note this is an experimental tool. It performs well on the real flight logs provided, but there may still be bugs.

## Compiling (optional)

A pre-compiled executable **is already provided**, but you can compile it yourself if you wish, in your favourite terminal:

`make`

Your compiler must support the C++20 standard.

Please note this has been tested on Windows using the MinGW-w64 compiler (you can get it [here](https://www.msys2.org/)). No Windows-specific libraries are used, so porting to other systems should be simple. Pull requests are welcome.

## Running

In your favourite terminal, run:

`./hlat.exe <example.csv>`

and replace `<example.csv>` with a log file of your choosing. Several sample log files are available in the `logs` directory. These are also useful for testing.

## Questions?

If you have any questions, please use Midnight Make's [contact form](https://www.midnightmake.com/pages/contact), or email [geoff@midnightmake.com](mailto:geoff@midnightmake.com).

Pull requests are welcome.
