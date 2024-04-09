"""
This script helps to split a static library code from application code
and allows to flash them independently.

It does so by creating a new assembly module contains locations of all
global symbols from the `.map` file of the static library.
It also intialization code that takes care of the constant initialization
in the static module.
"""
import re
import sys

mapfile =  open(sys.argv[1]).read()
symmap = {m.group('name'): m.group('addr')
          for m in re.finditer(r'(?P<addr>[0-9A-F]+)  (?P<name>\w+)', mapfile)}

print('.module staticlib')
for k in symmap:
  if k.startswith('_'):
    print(f'  .globl {k}')

print(f'''
  .globl __xdata_init
  .globl __xdata_init_start
  .globl __xdata_init_len

.area CODE
  __xdata_init=0x{symmap['s_INITIALIZER']}
  __xdata_init_start=0x{symmap['s_INITIALIZED']}
  __xdata_init_len=0x{symmap['l_INITIALIZER']}
''')
for k, v in symmap.items():
  if k.startswith('_'):
    print(f'  {k}=0x{v}')

print('''
.area GSINIT
   ldw x, #__xdata_init_len
loop$:
   ld a, (__xdata_init - 1, x)
   ld (__xdata_init_start - 1, x), a
   decw x
   jrne loop$
done$:
''')

open('static.lib.datastart', 'w').write(hex(int(symmap['s_INITIALIZED'], 16) + int(symmap['l_INITIALIZED'], 16)))
