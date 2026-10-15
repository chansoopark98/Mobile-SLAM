"""Independent assembler controls for the bounded ARM64 xref decoder."""
import importlib.util
import pathlib
import subprocess
import tempfile
import unittest

SCRIPT = pathlib.Path(__file__).with_name('binary-boundaries.py')
spec = importlib.util.spec_from_file_location('binary_boundaries', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class AddressControls(unittest.TestCase):
    def check_layout(self, rodata_address):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            assembly = root / 'control.s'
            assembly.write_text('''.text
.global _start
_start:
 adrp x2, target
 add x5, x2, :lo12:target
 adrp x7, target
 add x8, x6, :lo12:target
 adrp x9, target
 nop
 add x9, x9, :lo12:target
 ret
.section .rodata
.global target
target:
.asciz "independent-address-control"
''')
            subprocess.run(['aarch64-linux-gnu-as', str(assembly), '-o', str(root / 'control.o')], check=True, capture_output=True)
            subprocess.run(['aarch64-linux-gnu-ld', '--no-relax', '-Ttext=0x20000',
                            f'--section-start=.rodata={rodata_address:#x}', str(root / 'control.o'),
                            '-o', str(root / 'control')], check=True, capture_output=True)
            data = (root / 'control').read_bytes()
            sections = module.elf_sections(data)
            self.assertEqual(sections['.rodata']['address'], rodata_address)
            refs = module.adjacent_adrp_add(data, sections['.text'])
            self.assertEqual(refs, [{'instruction_address': 0x20000, 'target_address': rodata_address,
                                     'adrp_register': 2, 'add_destination': 5}])
            text = subprocess.check_output(['aarch64-linux-gnu-objdump', '-d', str(root / 'control')], text=True)
            self.assertIn(f'{rodata_address:x} <target>', text)

    def test_negative_page_delta_and_mismatched_decoys(self):
        self.check_layout(0x10000)

    def test_positive_page_delta_and_mismatched_decoys(self):
        self.check_layout(0x30000)


if __name__ == '__main__':
    unittest.main()
