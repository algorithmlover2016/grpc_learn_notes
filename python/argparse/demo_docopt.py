"""Compute vanishing points from images.
Usage:
    demo_docopt.py [options] <yaml-config> <checkpoint> <image>...
    demo_docopt.py ( -h | --help )

Arguments:
   <yaml-config>                 Path to the yaml hyper-parameter file
   <checkpoint>                  Path to the checkpoint
   <image>...                    Path to an image

Options:
   -h --help                     Show this screen
   -d --devices <devices>        Comma seperated GPU devices [default: 0]
   -o --output <output>          Path to the output AA curve [default: error.npz]
   --dump <output-dir>           Optionally, save the vanishing points to npz format.[default: output]
"""

from re import I
from docopt import docopt

# refer to https://www.geeksforgeeks.org/docopt-module-in-python/
# docopt(doc, argv = None, version = None, help = True, options = False)
doc ='''
Usage:
    
  demo_docopt.py  [(<name1>|<name2>)] <name3>...
  demo_docopt.py  mov <name1> <name2>
  demo_docopt.py  (--h|--q) [<name1> -l]
  demo_docopt.py  --version
  
Options:
  
  -l, --all      List all.
  -q, --quit     exit.
  --version      Version 3.6.1 
  -h --help      Show this screen.
  --version      Show version.
  --speed =<kn>   Speed in knots [default: 10].
  --moored       Moored (anchored) mine.
  --drifting     Drifting mine.   
'''  
    
from docopt import docopt
# args1 = docopt(doc)
# print(args1)

# demo command: python .\demo_docopt.py  -d 1 ffff.cfg sfsfds.tar.gz llsd fdasfdsaf fsdaafasdfs fasddafadsf
args = docopt(__doc__)
print(args)
config_file = args["<yaml-config>"]
num_gpus = args["--devices"].count(",") + 1    
if args["<checkpoint>"]:
    print(args["<checkpoint>"])

if args["--dump"] is not None:
    print(args["--dump"])

if args["<image>"] is not None:
    print(args["<image>"])
