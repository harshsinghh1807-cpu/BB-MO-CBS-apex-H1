#!/usr/bin/env python3
"""Exercise the built CLI, real input parsing, JSON output and error handling."""
from pathlib import Path
import subprocess
import sys
import tempfile
from check_output import validate

binary = Path(sys.argv[1]).resolve()
root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='pex-h-') as directory:
    temp = Path(directory)
    base = [str(binary), '-m', str(root/'example/tiny.map'), '-n', '2', '-d', '2',
            '-s', str(root/'example/tiny.scen'), '--c1', str(root/'example/tiny-1.cost'),
            '--c2', str(root/'example/tiny-2.cost'), '-o', str(temp/'output.txt'),
            '--paths_file', str(temp/'paths.json'), '-t', '10']
    for extra in [[], ['-e', '0.12345678912345678'], ['-a', 'BBMOCBS-pex'], ['-a', 'BBMOCBS-pex-H1'], ['-e', '0.1', '-K', '2', '--CB', 'true'],
                  ['-a', 'BBMOCBS-apex-H1', '--turn_dim', '0', '--turn_cost', '2']]:
        result = subprocess.run(base+extra, capture_output=True, text=True, timeout=15)
        assert result.returncode == 0, result.stderr
        data = validate(root/'example/tiny.map', root/'example/tiny.scen',
                        [root/'example/tiny-1.cost', root/'example/tiny-2.cost'],
                        temp/'paths.json', 2, 0 if '--turn_dim' in extra else -1, 2)
        assert data['completed'] and data['solutions']
        if '-e' in extra:
            assert data['epsilon'] == float(extra[extra.index('-e')+1])
    (temp/'time.cost').write_text('1 1 1\n1 1 1\n')
    for algorithm in ['BBMOCBS-pex','BBMOCBS-pex-H1']:
        command=list(base)
        command[command.index('-d')+1]='3'
        command += ['--c3',str(temp/'time.cost'),'-a',algorithm]
        run=subprocess.run(command,capture_output=True,text=True,timeout=15)
        assert run.returncode==0,run.stderr
        data=validate(root/'example/tiny.map',root/'example/tiny.scen',
                      [root/'example/tiny-1.cost',root/'example/tiny-2.cost',temp/'time.cost'],temp/'paths.json',2)
        assert data['completed'] and data['objectives']==3
    for option, value in [('-K', '0'), ('-e', '-0.1'), ('-e', 'nan'),
                          ('-n', '0'), ('-d', '4'), ('--eager', 'true'), ('--CB', 'yes'),
                          ('--turn_cost', '-1')]:
        command = list(base)
        if option in command:
            command[command.index(option)+1] = value
        else:
            command += [option, value]
        result = subprocess.run(command, capture_output=True, text=True, timeout=5)
        assert result.returncode == 1 and 'Error:' in result.stderr, (option, result.stderr)
    # Malformed map must raise a normal error, including destruction of an unread Map.
    (temp/'bad.map').write_text('type octile\nheight 2\nwidth 3\nmap\n..\n...\n')
    command = list(base); command[command.index('-m')+1] = str(temp/'bad.map')
    result = subprocess.run(command, capture_output=True, text=True, timeout=5)
    assert result.returncode == 1 and 'row width' in result.stderr
    command = list(base);command[command.index('-n')+1] = '3'
    result = subprocess.run(command, capture_output=True, text=True, timeout=5)
    assert result.returncode == 1 and 'fewer valid agents' in result.stderr
print('PASS: CLI examples, path/cost validation and invalid-input checks')
