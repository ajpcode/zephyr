# Copyright (c) 2024 Curtis Instruments, Inc.
#
# SPDX-License-Identifier: Apache-2.0

"""Runner for flashing and debug with STM32CubeIDE tools, the official programming
   and debug utilities from ST Microelectronics. (Linux only at the moment)
"""

import argparse
from pathlib import Path
import platform
import os
import shlex
import shutil
from typing import List, Optional, ClassVar, Dict

from runners.core import ZephyrBinaryRunner, RunnerCaps, RunnerConfig
import subprocess

def _find_files_recursive(path, pattern) -> [Path]:
    return [p for p in Path(path).rglob(pattern)]

def _find_subfolder(root_dir, subfolder_name):
    root_path = Path(root_dir)
    for path in root_path.rglob(subfolder_name):
        if path.is_dir():
            return path
    print( f"Failed to find {subfolder_name }" )
    return None

def _get_west_config(key):
    result = subprocess.run(['west', 'config', key], capture_output=True, text=True)
    return result.stdout.strip()

board_root = str( os.getenv('BOARD_ROOT') )
board_name = str( _get_west_config('build.board') )

print( "****** " + board_root + " - " + board_name + " *****" )

class STM32CubeIDEBinaryRunner(ZephyrBinaryRunner):
    """Runner front-end for STM32CubeIde tools."""

    _RESET_MODES: ClassVar[Dict[str, str]] = {
        "sw": "SWrst",
        "hw": "HWrst",
        "core": "Crst",
    }
    """Reset mode argument mappings."""

    def __init__(
        self,
        cfg: RunnerConfig,
        port: str,
        frequency: Optional[int],
        reset_mode: Optional[str],
        conn_modifiers: Optional[str],
        cli: Optional[Path],
        gdbserver: Optional[Path],
        use_elf: bool,
        erase: bool,
        extload: Optional[str],
        tool_opt: List[str]
    ) -> None:
        super().__init__(cfg)

        self._port = port
        self._frequency = frequency
        self._reset_mode = reset_mode
        self._conn_modifiers = conn_modifiers
        self._cli = (
            cli or STM32CubeIDEBinaryRunner._get_stm32cubeprogrammer_path(self.logger)
        )
        self._debugserver = (
            gdbserver or STM32CubeIDEBinaryRunner._get_stlink_gdbserver_path(self.logger)
        )
        self._use_elf = use_elf
        self._erase = erase
        self._gdb_cmd = [cfg.gdb] if cfg.gdb is not None else None
        self._elf_name = cfg.elf_file

        if extload:
            board_path = _find_subfolder( board_root, board_name.split('/')[0] )
            p = Path()
            self.logger.info( f"Looking in: {board_path}, for: {extload}" );
            if Path(os.path.join(board_path, extload)).exists():
                p = board_path
                self.logger.info( f"Found: {p}" );
            else:
                self.logger.warn( f"Unable to find loader ({extload}) in boards directory ({board_path}), reverting to cube programmer loader path." );
                p = STM32CubeIDEBinaryRunner._get_stm32cubeprogrammer_path(self.logger).parent.resolve() / 'ExternalLoader'
            self._extload = ['-el', str(p / extload)]
        else:
            self._extload = []

        self._tool_opt: List[str] = list()
        for opts in [shlex.split(opt) for opt in tool_opt]:
            self._tool_opt += opts

        # add required library loader path to the environment (Linux only)
        if platform.system() == "Linux":
            os.environ["LD_LIBRARY_PATH"] = str(self._cli.parent / ".." / "lib")

    @staticmethod
    def _get_stm32cubeprogrammer_path(logger) -> Path:
        """Obtain path of the STM32CubeProgrammer CLI tool."""

        cmd = shutil.which("STM32_Programmer_CLI")
        if cmd is not None:
            return Path(cmd)

        if platform.system() == "Linux":
            # Look in the CUBE IDE default install path
            programmer_list = _find_files_recursive("/opt/st/", "STM32_Programmer_CLI")
            programmer_list_len = len( programmer_list )
            if programmer_list_len > 0:
                programmer_list_0 = str( programmer_list[0] )
                if programmer_list_len > 1:
                    logger.warn(f'Found more than one STM32_Programmer_CLI installation, using the first one: {programmer_list_0}')
                return programmer_list[0]

        logger.warn("Could not determine STM32_Programmer_CLI path")
        return None

    @staticmethod
    def _get_stlink_gdbserver_path(logger) -> Path:
        cmd = shutil.which("ST-LINK_gdbserver")
        if cmd is not None:
            return Path(cmd)

        if platform.system() == "Linux":
            gdbserver_list = _find_files_recursive("/opt/st/", "ST-LINK_gdbserver")
            gdbserver_list_len = len( gdbserver_list )
            if gdbserver_list_len > 0:
                gdbserver_list_0 = str( gdbserver_list[0] )
                if gdbserver_list_len > 1:
                    logger.warn(f'Found more than one ST-LINK_gdbserver installation, using the first one: {gdbserver_list_0}')
                return gdbserver_list[0]

        logger.warn("Could not determine ST-LINK_gdbserver path")
        return None

    @classmethod
    def name(cls):
        return "stm32cubeide"

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={"flash", "debug", "debugserver"}, erase=True, extload=True, tool_opt=True)

    @classmethod
    def do_add_parser(cls, parser):
        parser.add_argument(
            "--port",
            type=str,
            required=True,
            help="Interface identifier, e.g. swd, jtag, /dev/ttyS0...",
        )
        parser.add_argument(
            "--frequency", type=int, required=False, help="Programmer frequency in KHz"
        )
        parser.add_argument(
            "--reset-mode",
            type=str,
            required=False,
            choices=["sw", "hw", "core"],
            help="Reset mode",
        )
        parser.add_argument(
            "--conn-modifiers",
            type=str,
            required=False,
            help="Additional options for the --connect argument",
        )
        parser.add_argument(
            "--cli",
            type=Path,
            required=False,
            help="STM32CubeProgrammer CLI tool path",
        )
        parser.add_argument(
            "--gdbserver",
            type=Path,
            required=False,
            help="ST-LINK_gdbserver tool path",
        )
        parser.add_argument(
            "--use-elf",
            action="store_true",
            required=False,
            help="Use ELF file when flashing instead of HEX file",
        )

    @classmethod
    def extload_help(cls) -> str:
        return "External Loader for STM32_Programmer_CLI"

    @classmethod
    def tool_opt_help(cls) -> str:
        return "Additional options for STM32_Programmer_CLI"

    @classmethod
    def do_create(
        cls, cfg: RunnerConfig, args: argparse.Namespace
    ) -> "STM32CubeIDEBinaryRunner":
        return STM32CubeIDEBinaryRunner(
            cfg,
            port=args.port,
            frequency=args.frequency,
            reset_mode=args.reset_mode,
            conn_modifiers=args.conn_modifiers,
            cli=args.cli,
            gdbserver=args.gdbserver,
            use_elf=args.use_elf,
            erase=args.erase,
            extload=args.extload,
            tool_opt=args.tool_opt,
        )

    def do_run(self, command: str, **kwargs):
        if command == "flash":
            self.flash(**kwargs)
        else:
            self.debug_debugserver(command, **kwargs)

    def flash(self, **kwargs) -> None:
        self.require(str(self._cli))

        # prepare base command
        cmd = [str(self._cli)]

        connect_opts = f"port={self._port}"
        if self._frequency:
            connect_opts += f" freq={self._frequency}"
        if self._reset_mode:
            reset_mode = STM32CubeIDEBinaryRunner._RESET_MODES[self._reset_mode]
            connect_opts += f" reset={reset_mode}"
        if self._conn_modifiers:
            connect_opts += f" {self._conn_modifiers}"

        cmd += ["--connect", connect_opts]
        cmd += self._tool_opt
        if self._extload:
            # external loader to come after the tool option in STM32CubeProgrammer
            cmd += self._extload

        # erase first if requested
        if self._erase:
            self.check_call(cmd + ["--erase", "all"])

        # flash image and run application
        dl_file = self.cfg.elf_file if self._use_elf else self.cfg.hex_file
        if dl_file is None:
            raise RuntimeError('cannot flash; no download file was specified')
        elif not os.path.isfile(dl_file):
            raise RuntimeError(f'download file {dl_file} does not exist')
        self.check_call(cmd + ["--download", dl_file, "--start"])


    def log_gdbserver_message(self):
        self.logger.info('ST-LINK gdb server running on port {}'.
                         format(self.gdb_port))


    def debug_debugserver(self, command: str, **kwargs) -> None:
        if( None ==self._debugserver ):
            raise RuntimeError('Debugging unavailable - No gdbserver assigned, see warning messages for info.')

        self.gdb_port = 61234

        if self._cli.exists():
            self.logger.info( f'STM32 Programmer executable found: {str(self._cli)}' )
        else:
            self.logger.info( f'STM32 Programmer executable NOT found ({str(self._cli)})' )
            raise ValueError('Requies STM32_Programmer_CLI to be available to enable programming of device pior to debugging.')

        if self._debugserver.exists():
            self.logger.info( f'STM32 gdbserver executable found: {str(self._debugserver)}' )
        else:
            self.logger.warn( f'STM32 gdbserver executable NOT found ({str(self._debugserver)})' )
            raise ValueError('Requie ST-LINK_gdbserver to be available to enable debugging')

        if self._extload[1] != None:
            ext_ldr: Path = Path( self._extload[1] )
            if ext_ldr.exists():
                self.logger.info( f'External loader executable found: {str(self._extload[1])}' )
            else:
                self.logger.warn( f'External loader executable NOT found: {str(self._extload[1])}' )
                raise ValueError('Requie external loader to be available to enable debugging')

        # Prepare base command
        server_cmd = [str(self._debugserver),
            '-e',
            '-s',
            # Name of log file
            '-f', 'debug.log',
            # Port number for GDB to connect on
            '-p', f'{self.gdb_port}',
            # Refresh delay
            '-r', '15',
            # Enable SWD mode
            '-d',
            # Path to STM32_Programmer_CLI (directory, not binary)
            '-cp', os.path.dirname(self._cli),
            # External Loader to use
            self._extload[0],
            self._extload[1] ]

        self.flash(**kwargs)

        # Debug gdbserver string
        if command == 'debugserver':
            self.log_gdbserver_message()
            self.check_call(server_cmd)

        elif command == 'debug':
            client_cmd = (self._gdb_cmd +
                          [self._elf_name] +
                          ['-ex', f'target remote :{self.gdb_port}'])
            client_cmd += ['-ex', 'b main',
                           '-ex', 'c' ]

            self.log_gdbserver_message()
            self.run_server_and_client(server_cmd, client_cmd)
        else:
            ValueError('Only the commands: flash, debug and debugserver are allowed')

