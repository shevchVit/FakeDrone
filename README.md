# Задание 2 FakeDrone

## FakeDrone

Программа может принимать следующие команды типа COMMAND_LONG:
- MAV_CMD_COMPONENT_ARM_DISARM
- MAV_CMD_NAV_TAKEOFF
- MAV_CMD_NAV_LAND

Также принимаются и отправляются HEARTBEAT'ы.
При отправке телеметрии генерируются случайные значения.

Запуск программы:
```
python FakeDrone.py --port=5656
```
Без указания порта mavlink будет использовать порт 5656.

## CommandSender

Эта программа - пример взаимодействия с FakeDrone.


