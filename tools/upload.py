import argparse
import asyncio
import subprocess
import sys

from pybricksdev.ble import find_device
from pybricksdev.connections.pybricks import PybricksHubBLE


async def upload(nome, arquivo, start=True, wait=True):
    print(f"Conectando a {nome}...")
    try:
        dispositivo = await find_device(nome)
    except TimeoutError:
        print("Tempo excedido. Verifique que o spike está ligado e tente novamente.", file=sys.stderr)
        return 1

    hub = PybricksHubBLE(dispositivo)
    await hub.connect()
    try:
        if start:
            await hub.run(arquivo, wait=wait, print_output=True)
        else:
            await hub.download(arquivo)
    finally:
        await hub.disconnect()
    return 0


def report_compile_error(error):
    print("Falha ao compilar o programa:", file=sys.stderr)
    for output in (error.stdout, error.stderr):
        if output:
            if isinstance(output, bytes):
                output = output.decode(errors="replace")
            print(output.rstrip(), file=sys.stderr)
    if not error.stdout and not error.stderr:
        print(error, file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description="Envia um programa ao hub Pybricks por BLE.")
    parser.add_argument("nome", help="Nome Bluetooth do hub")
    parser.add_argument("arquivo", help="Arquivo Python a enviar")
    parser.add_argument("--no-start", action="store_true", help="Somente envia, sem executar")
    parser.add_argument("--no-wait", action="store_true", help="Desconecta sem esperar o programa terminar")
    args = parser.parse_args()

    try:
        return asyncio.run(
            upload(args.nome, args.arquivo, start=not args.no_start, wait=not args.no_wait)
        )
    except subprocess.CalledProcessError as error:
        report_compile_error(error)
        return error.returncode or 1
    except KeyboardInterrupt:
        print("Cancelado")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
