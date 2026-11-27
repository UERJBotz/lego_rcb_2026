# Carcará Branco / Carrara Táxi Carrara / Renato Sorriso / O Garra

Repositório criado para o desafio RCB Challenge da CBR 2026 (antigo SEK).


## Pré-requisitos

### Hardware

Usamos:
- 2 Hubs SPIKE Prime, da LEGO (com o firmware do pybricks);
- 1 ESP32 (com micropython);

Outro dispositivo da família esp32 provavelmente funcionaria (provalmente tendo que alterar alguns pinos) contando que tenha suporte a bluetooth low energy (ble).

### Software

Todo o desenvolvimento do projeto foi feito no Ubuntu 24.04.
Não recomendamos o uso do windows. Nenhum outro sistema foi testado.

Certifique-se de ter o Python 3 instalado na sua máquina, na versão 3.9 ou maior. Você pode verificar a versão do Python com o comando:

```bash
python --version
```

O script de upload é uma Makefile, então é necessário instalar o make no sistema. Foi testada apenas a versão gnu.

No ubuntu:
```bash
sudo apt install make
```

Para o código dos spikes, fazemos upload com o pybricksdev e para o esp32 com o ampy.


## Configurando o Ambiente

Essa seção é baseada no [tutorial oficial da pybricks](https://pybricks.com/project/pybricks-other-editors/) sobre o uso de outros editores.

Este projeto utiliza um ambiente virtual para gerenciar dependências de forma isolada. Siga as instruções abaixo para configurar o ambiente.


### Passo 1: Crie o Ambiente Virtual

Crie um ambiente virtual na pasta do projeto (normalmente chamado de `.venv`):

```bash
python -m venv .venv
```


### Passo 2: Instale as Dependências

Instale as dependências do projeto listadas no arquivo `requirements.txt`:

No linux:
```bash
.venv/bin/pip install -r requirements.txt
```
No Windows:
```bash
.venv/Scripts/pip install -r requirements.txt
```

Agora seu ambiente virtual está configurado e pronto para uso!


## Executando o Projeto

Escreva `make` no terminal para fazer upload para o dos dois spikes.
O script espera que você tenha um chamado spike0 (o "braço") e outro chamado spike1 (a "cabeça").
Isso pode ser configurado com uma variável na makefile.

Para fazer upload do código para os microcontroladores, plugue primeiro o esp32, depois o arduino.
Então rode `make rabo` no terminal.
O script espera que estejam nas portas `/dev/ttyACM0` e `/dev/ttyACM1`, respectivamente (não funciona no windows).


## Contribuição

### Interna
Se desejar contribuir com o projeto, faça um clone do repositório, crie uma branch com a sua feature, e envie um push para a sua branch. **Não comite para a master** 

1. Faça o clone do projeto (`git clone https://github.com/UERJBotz/lego_rcb_2024.git`)
2. Crie uma branch para sua feature (`git switch -c nome-da-sua-branch`)
3. Adicione suas mudanças (`git add $arquivo1 $arquivo2 ... $arquivoN`)
4. Comite suas mudanças explicando o que/porque mudou (`git commit`) com uma mensagem descritiva.
5. Faça o push para a branch (`git push origin nome-da-sua-branch`)
6. Abra um Pull Request


### Externa 
1. Faça o fork do projeto
2. Siga os passos para a contribuição interna

Qualquer dúvida, entre em contato ou abra uma issue no repositório.
