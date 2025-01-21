# TechFab ESP Framework LoRa

> Baseado [neste](https://github.com/nopnop2002/esp-idf-sx127x) repositório.

## Dispositivos
Driver SX1276/77/78/79 para esp-idf.

## Requisitos
ESP-IDF V5.2 ou superior.   

# Uso

__Nota para ESP32C3__   
Por algum motivo, há placas de desenvolvimento que não podem usar GPIO06, GPIO08, GPIO09 e GPIO019 para pinos clock SPI.
De acordo com as espeficiações, esses pinos também podem ser usados como clocks SPI.
Foi usado um ESP-C3-13 para verificar se esses pinos poderiam ser usados.

# Configuração para o Transceptor   

![config-lora-1](https://user-images.githubusercontent.com/6020549/152313802-d88ed3ab-dff5-4fe5-a05f-742c2e6e0aa4.jpg)

## Frequência usada
![config-lora-2](https://github.com/user-attachments/assets/91c4b8b8-e18c-4dbb-b880-40d2cd460272)

## Usando transceptor além dos 433MHz / 866MHz / 915MHz   
![config-lora-3](https://github.com/user-attachments/assets/62984a47-681e-48f4-a408-d8429fceea58)

## Configurações avançadas   
![config-lora-4](https://github.com/user-attachments/assets/513f7bca-63ea-4045-a517-8de054fbb804)

A comunicação LoRa tem os três parâmetros de comunicação seguintes.
1.Error Coding Rate (= CR)   
2.Signal Bandwidth (= BW)   
3.Spreading Factor (= SF)   
A velocidade de comunicação é mais rápida quando BW é grande, CR e SF são pequenos.
Entretanto, a medida que a velocidade de comunicação aumenta, a RSSI (sensibilidade a recepção) deteriora, então escolha uma que se adeque as suas necessidades.

- Error coding rate   
1:4/5(Default)   
2:4/6   
3:4/7   
4:4/8   

- Signal Bandwidth   
0:7.8 kHz   
1:10.4 kHz   
2:15.6 kHz   
3:20.8kHz   
4:31.25 kHz   
5:41.7 kHz   
6:62.5 kHz   
7:125 kHz(Default)   
8:250 kHz   
9:500 kHz   
Na banda mais baixa (169MHz), BWs 8&9 não são suportados.

- Spreading Factor (expressos como logaritmo de base 2)   
6:64 chips / symbol   
7:128 chips / symbol(Default)   
8:256 chips / symbol   
9:512 chips / symbol   
10:1024 chips / symbol   
11:2048 chips / symbol   
12:4096 chips / symbol   

- Seleção SPI BUS
![config-lora-5](https://github.com/user-attachments/assets/f3dcf76e-1bf4-4c05-98ac-f9174f52820e)


# Comunicação com SX126X
O formato do pacote lora é estritamente especificado.
Logo, se os parâmetros abaixo forem iguais, eles podem se comunicar.
- Signal Bandwidth (= BW)   
- Error Coding Rate (= CR)   
- Spreading Factor (= SF)   

# Sobre velocidade de comunicação e RSSI
Na modulação LoRa, a velocidade de comunicação (bps) e sensitividade máxima de recepção (RSSI) são determinados pela combinação do fator de espalhamento (SF), banda (BW) e taxa de erro (CR).
- SF   
Aumentar o fator de espalhamento aumenta à resistência a ruido
Isso melhora o RSSI e aumenta o alcance da comunicação, mas diminui a velocidade.

- BW   
Aumentando a largura de banda de comunicação aumenta a velocidade.
Entretando, o RSSI irá diminuir.

- CDR ou CR
Quanto maior for a taxa de correção de erros, melhor a correção, porém diminui a quantidade de informação por pacote.
(Sem efeito no RSSI)
Você pode escolher se usa Optimaise para cada CR, aprimorando a taxa de correção, mas reduzindo a velocidade de comunicação.

# Velocidade com diferentes BW em Bytes/Sec
ESP32@160/433MHz/CR=1/SF=7   
|BW=0(7.8KHz)|BW=2(15.6KHz)|BW=4(31.25KHz)|BW=6(62.5KHz)|BW=7(125KHz)|BW=8(250KHz)|BW=9(500KHz)|
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
|37.50|83.18|165.75|296.32|618.93|1153.85|2031.87|

# Velocidade com diferentes CR em Bytes/Sec   
ESP32@160/433MHz/BW=7/SF=7   
|CR=1(4/5)|CR=2(4/6)|CR=3(4/7)|CR=4(4/8)|
|:-:|:-:|:-:|:-:|
|618.93|543.13|468.32|410.63|

# Datasheet
Datasheet está [aqui](https://github.com/jgromes/RadioLib/files/8646997/DS_SX1276-7-8-9_W_APP_V7.pdf).   

# Reference
https://github.com/techfab-iot/tef-lora

