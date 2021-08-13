#-*- coding:utf-8 -*-

import argparse

parser = argparse.ArgumentParser(description='Process some integers. Display before paramter helping information', epilog = 'Display after parameters helping information')
# modify integers into ints to verify that the str in the position is just a class property
parser.add_argument('ints', metavar='N', type=int, nargs='+',
                    help='an integer for the accumulator of the %(prog)s program')
parser.add_argument('--sum', dest='accumulate', action='store_const',
                    const=sum, default=max,
                    help='sum the integers (default: find the max)')

args = parser.parse_args()
print(args.accumulate(args.ints))



actionparser = argparse.ArgumentParser()
actionparser.add_argument('--foo', action='store_true')
actionparser.add_argument('--bar', action='store_false')
actionparser.add_argument('--baz', action='store_false')

print(actionparser.parse_args('--foo --bar'.split()))

args = actionparser.parse_args('--bar --baz'.split())
print(args)
print(args.bar, args.baz)

args = actionparser.parse_args('--foo'.split())
print(args.foo, args.bar, args.baz)

nargsparser = argparse.ArgumentParser()
nargsparser.add_argument('--foo')
nargsparser.add_argument('comnand')
nargsparser.add_argument('args', nargs = argparse.REMAINDER)
print(nargsparser.parse_args('--foo B cmd --arg1 XX ZZ'.split()))
print(nargsparser.parse_args('cmd --foo B --arg1 XX ZZ'.split()))
