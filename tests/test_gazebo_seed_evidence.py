import tempfile
import unittest
from pathlib import Path

from validate_autonomous_search import gazebo_seed_evidence


class GazeboSeedEvidenceTests(unittest.TestCase):
    def test_requires_unique_actual_server_seed_world_and_lockstep(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            server = root/'123'; server.mkdir()
            world = root/'world.sdf'; world.touch()
            def write(args):
                (server/'cmdline').write_bytes(b'\0'.join(x.encode() for x in args)+b'\0')
            args = ['gzserver', '--lockstep', '--seed', '17', str(world)]
            write(args)
            self.assertEqual(gazebo_seed_evidence(17, world, root)['pid'], 123)
            for bad in (['gzserver', '--lockstep', str(world)],
                        ['gzserver', '--lockstep', '--seed', '18', str(world)],
                        ['gzserver', '--seed', '17', str(world)],
                        ['gzserver', '--lockstep', '--seed', '17', '/other.world'],
                        ['gzserver', '--lockstep', '--seed', '17', '--seed', '18', str(world)]):
                with self.subTest(bad=bad):
                    write(bad)
                    with self.assertRaises(RuntimeError): gazebo_seed_evidence(17, world, root)
            write(args)
            other = root/'124'; other.mkdir(); (other/'cmdline').write_bytes((server/'cmdline').read_bytes())
            with self.assertRaises(RuntimeError): gazebo_seed_evidence(17, world, root)


if __name__ == '__main__':
    unittest.main()
