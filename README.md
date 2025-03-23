# Паяльная станция

## Описание
Программное обеспечение для управления паяльной станцией с инфракрасным столом и двумя паяльными нагревателями. Приложение позволяет контролировать температуру верхнего и нижнего нагревателей, а также преднагревателя, выбирать профили пайки и управлять процессом удаленно через Serial или WiFi соединение.

## Функциональные возможности
- Подключение к паяльной станции через Serial-порт или WiFi
- Мониторинг температуры трех нагревателей в режиме реального времени:
  - Верхний нагреватель
  - Нижний нагреватель
  - Преднагреватель
- Выбор предустановленных профилей пайки с различными температурными режимами
- Запуск и остановка процесса пайки
- Графический интерфейс пользователя на PyQt5

## Требования
- Python 3.6 или выше
- PyQt5
- pyserial (для Serial-подключения)
- socket (стандартный модуль Python, для WiFi-подключения)

## Установка
```
pip install PyQt5 pyserial
```

## Использование
1. Запустите приложение:
```
python app_py/gui.py
```

2. Выберите тип подключения:
   - Serial: укажите COM-порт (например, COM3)
   - WiFi: укажите IP-адрес паяльной станции

3. Нажмите кнопку "Connect" для установки соединения

4. После успешного подключения вы увидите текущие показания температуры всех нагревателей

5. Выберите профиль пайки из выпадающего списка:
   - Профиль 1: Верхний нагреватель - 160°C, Нижний нагреватель - 170°C, Преднагреватель - 180°C
   - Профиль 2: Верхний нагреватель - 170°C, Нижний нагреватель - 180°C, Преднагреватель - 190°C
   - Профиль 3: Верхний нагреватель - 180°C, Нижний нагреватель - 190°C, Преднагреватель - 200°C

6. Нажмите кнопку "Старт" для начала процесса пайки

7. При необходимости нажмите кнопку "Стоп" для остановки процесса

## Структура проекта
- `app_py/gui.py` - Основной файл с графическим интерфейсом
- `app_py/connection.py` - Модуль для управления подключениями (Serial и WiFi)
- `app_py/commands.py` - Модуль с командами для управления паяльной станцией
- `app_py/temperature.py` - Модуль для работы с температурными данными
- `fw_ESP32-S2-DevKitM-1/` - Прошивка для ESP32

## Лицензия
См. файл [LICENSE](LICENSE) для дополнительной информации.
