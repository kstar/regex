# regex
Regular expression engine

An attempt at writing a regular expression engine that uses the Thomson NFA approach, mostly for educative reasons.
The code follows Russ Cox's exposé for the most parts. The machine code compiled is specific to Intel x86_64 on Linux (System V ABI)
The Regex class also allows for visualization of the NFA or DFA by outputting the graphviz DOT format code to represent the graph.
