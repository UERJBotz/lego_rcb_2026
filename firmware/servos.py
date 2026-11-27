from time import sleep_ms


class ServoController:
    """Controla os seis servos do controlador serial usado pela placa de motores."""

    BAUDRATE = 250000
    NUM_SERVOS = 6
    DISABLED = 200
    COMMAND = 0xFD

    def __init__(self, uart=None, uart_id=1, tx=None, rx=None, timeout=1000):
        if uart is None:
            if tx is None or rx is None:
                raise ValueError("Informe tx e rx para configurar a UART")
            from machine import UART

            uart = UART(
                uart_id,
                baudrate=self.BAUDRATE,
                tx=tx,
                rx=rx,
                timeout=timeout,
            )

        self.uart = uart
        self.posicoes = [self.DISABLED] * self.NUM_SERVOS

    def _validar_servo(self, servo):
        if not isinstance(servo, int) or not 1 <= servo <= self.NUM_SERVOS:
            raise ValueError("O servo deve estar entre 1 e 6")

    def atualiza_servos(self):
        while self.uart.any():
            self.uart.read()

        quadro = bytearray([self.COMMAND] + self.posicoes + [0, 0, 0])
        escrito = self.uart.write(quadro)
        if escrito is not None and escrito != len(quadro):
            raise OSError("Envio incompleto do quadro de servos")

        resposta = self.uart.read(1)
        if resposta != bytes([self.COMMAND]):
            raise OSError("Resposta inválida do controlador de servos")
        return True

    def _envia_posicao(self, indice, angulo):
        anterior = self.posicoes[indice]
        self.posicoes[indice] = angulo
        try:
            return self.atualiza_servos()
        except Exception:
            self.posicoes[indice] = anterior
            raise

    def move_servo(self, servo, angulo, tempo=0):
        self._validar_servo(servo)
        angulo = min(max(int(angulo), 0), 180)
        indice = servo - 1
        inicial = self.posicoes[indice]

        if tempo <= 0 or inicial == self.DISABLED or inicial == angulo:
            return self._envia_posicao(indice, angulo)

        passo = 1 if angulo > inicial else -1
        intervalo_ms = int(tempo * 1000 / abs(angulo - inicial))
        for atual in range(inicial + passo, angulo + passo, passo):
            self._envia_posicao(indice, atual)
            if atual != angulo and intervalo_ms > 0:
                sleep_ms(intervalo_ms)
        return True

    def desativa_servo(self, servo):
        self._validar_servo(servo)
        return self._envia_posicao(servo - 1, self.DISABLED)
