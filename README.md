			ПРОГРАММА: controller_RS485+BLE+DHT+AHG
	для контроллера внешнего устройства узла управления системы "Умный дом",
	соединенной с сеть с помощью интерфейса RS485. Протокол обмена Modbas.
	Управляет системой телефон/планшет на основе ОС Андроид. 
	Он периодически (4 сек.) опрашивает узлы системы через интерфейс RS485
	и анализируя сигналы от датчиков посылает сообщения через сотовую связь
	или включает исполнительные механизмы.	
	В данной версии размер команд на исполнение - 1 регистр.
	При считывании данных от датчиков запрашивается - 3 регистра начиная с нулевого адреса: 
	1 байт = состояние управляющих выходов ESP32;
	2 байт = состояние выбранных входов ESP32;
	3 байт = температура воздуха;
	4 байт = влажность воздуха;
	5 байт = влажность земли;
	6 байт = резерв.
ПРИМЕЧАНИЕ - если нет датчика в соответствующем байте 00 
ДАТЧИКИ
- IO22,вход, сигнал от датчика перемещений(лог.0). 
  разряд 0 в байте датчиков передаваемом при опросе по RS485;
- IO19,вход, зарезирвирован.
  разряд 1 в байте датчиков передаваемом при опросе по RS485;
- IO21,вход, зарезервирован.
  разряд 2 в байте датчиков передаваемом при опросе по RS485;
- IO5,вход, зарезирвирован.
  разряд 3 в байте датчиков передаваемом при опросе по RS485;
	
ВКЛЮЧЕНИЕ ИСПОЛНИТЕЛЬНЫХ МЕХАНИЗМОВ НЕПОСРЕДСТВЕННО
Вкл.\выкл. разряд 6 = 1\0 в старшем байте для управления передаваемом по RS485
Старший байт адреса регистра =0, передаваемом по RS485
- IO14,выход, вкл.\выкл. мотор.
  разряд 0 в мл.байте для управления передаваемом по RS485;
- IO27,выход, вкл.\выкл. свет.
  разряд 1 в мл.байте для управления передаваемом по RS485;
- IO13,выход, вкл.\выкл. вода.
  разряд 2 в мл.байте для управления передаваемом по RS485;
- IO26,выход, вкл.\выкл. сирена.
  разряд 3 в мл.байте для управления передаваемом по RS485;
- IO25,выход, вкл.\выкл. отопление.
  разряд 4 в мл.байте для управления передаваемом по RS485;
- IO2,выход, вкл.\выкл. розетка.
  разряд 5 в мл.байте для управления передаваемом по RS485.
ВКЛЮЧЕНИЕ ИСПОЛНИТЕЛЬНЫХ МЕХАНИЗМОВ УДАЛЕННО ПО BLE
Вкл.\выкл. исполнительных устройств произодится через посылку рекламы(адвертайзинга)
с узлового устройства сети RS485 расположенного в данном помещении.
 После общего наименования локальной сети следует номер передающего узла BLE.
 В полном локальном имени рекламы в конце три цифры означают команду\число как сумму кодов:
- Выкл. = 0;
- Вкл. = 256;
- вкл.\выкл. мотор. = 1;
- вкл.\выкл. свет. = 2;
- вкл.\выкл. вода. = 4;
- вкл.\выкл. сирена.= 8;
- вкл.\выкл. отопление. = 16;
- вкл.\выкл. розетка. =32;
	Пример: "MySmartHouse_06_258" - означает включить свет.
Передаваемые на узел сети по RS485 команды имеют следующий вид:
Старший байт адреса регистра =1, передаваемом по RS485
Вкл.\выкл. разряд 6 = 1\0 в старшем байте для управления передаваемом по RS485
- вкл.\выкл. мотор.
  разряд 0 в мл.байте для управления передаваемом по RS485;
- вкл.\выкл. свет.
  разряд 1 в мл.байте для управления передаваемом по RS485;
- вкл.\выкл. вода.
  разряд 2 в мл.байте для управления передаваемом по RS485;
- вкл.\выкл. сирена.
  разряд 3 в мл.байте для управления передаваемом по RS485;
- вкл.\выкл. отопление.
  разряд 4 в мл.байте для управления передаваемом по RS485;
- вкл.\выкл. розетка.
  разряд 5 в мл.байте для управления передаваемом по RS485.


		2 За основу взяты примеры:
	- работа UART ESP32 через интерфейс RS485:
https://github.com/espressif/esp-idf/tree/master/examples/peripherals/uart/uart_echo_rs485
	- работа с BLE "esp-idf/examples/bluetooth/hci/controller_vhci_ble_adv";
	- формирование рекламы (адвертайзинга) хорошо описано в: https://habr.com/ru/post/533580/
	- драйвер используемого цифрового датчика температуры и влажности DHT11 взят из библиотеки:
	https://github.com/UncleRus/esp-idf-lib;
 документация на библиотеку:
https://esp-idf-lib.readthedocs.io/en/latest/index.html

		3 Для формирования файла конфигурации указать Pyton расположение проекта, например,
cd C:\ESP32\Project\controller_RS485+BLE
	Задать чип ESP32 в качестве объекта для компилирования
idf.py set-target esp32

	Изменение тактовой частоты MIN 80MHz командой для компилятора:
"idf.py menuconfig" в настройках таблицы: component config/ESP_specific/

	Использование только одного ядра "PRO_CPU" командой для компилятора:
"idf.py menuconfig" в настройках таблицы:
component config/FreeRTOS/Run FreeRTOS only on first core
	
