# DroneSim

## Problem

Startups need easy and reliable testing soluutions

## Solution

Cloud service for iteration and AI feedback on results. Quick introduction of arguments and testing across various scenarios and conditiions

## !! DEAL BREAKERS !!

Companies have proprietary simulators

Companies can host their own solutions

Companies have already established devops chains

## Unfair advantages

Low competition in out of the box solutions

## Key metrics

Platform usage

Token usage growth

User feedback

## Finance

-> Token payment

<- Dev costs + hosting

## Channels and customers

Cooperation with defence clusters

Direct communicaton with startups

Defence tech startups (FPV only for MVP)


# DroneSim (MVP) — ArduPilot Copter SITL + Python сценарії + метрики + графіки (Windows)

Web-пайплайн для SIL (software-in-the-loop): прогін flight controller (ArduPilot Copter) у сим-середовищі через бібліотеку сценаріїв (імітації IMU/GPS/баро/магнітометр/RC + рухи), збір телеметрії, автоматичний розрахунок метрик стабільності/якості керування та візуалізація результатів.

Ціль MVP: **Windows**, **ArduPilot Copter SITL**, режим **GUIDED** + **SET_ATTITUDE_TARGET**, оркестрація на **Python**.

---

## 0) TL;DR (швидкий старт)

1) Підняти SITL у Docker і відкрити TCP 5760:
- `docker run ...`
- у контейнері: `sim_vehicle.py ... --out tcp:0.0.0.0:5760`

2) На Windows (у venv):
- `python src\check.py` → має бути `OK heartbeat`
- `python src\run_one_connect.py ...` → створить `telemetry.csv`
- `python src\analyze_run.py ...` → створить `metrics.json`
- `python src\visualize_run.py ...` → створить PNG-графіки (4 шт)

---

## 1) Передумови (Prerequisites)

### 1.1 Обов’язково
- Windows 10/11
- Docker Desktop (Linux containers)
- Python 3.x
- Git

### 1.2 Рекомендовано
- VS Code

### 1.3 Перевірки
```bat
docker --version
python --version
git --version
```

## 2) Структура репозиторію

DroneSim/
  src/
    check.py
    run_one_connect.py
    analyze_run.py
    visualize_run.py
  scenarios/
    step_roll_10deg.json
  params/
    good.json
  storage/
  Dockerfile.sitl (або Dockerfile)
  README.md

## 3) Налаштування на Windows
cd C:\Repos\DroneSim
python -m venv .venv
.\.venv\Scripts\activate
python -m pip install --upgrade pip
python -m pip install pymavlink numpy pandas matplotlib fastapi uvicorn
python -c "import sys; print(sys.executable)"
docker build -t ardupilot-sitl .

## 4) Перевірити, що образ є
docker images | findstr ardupilot-sitl

## 5) Запуск SITL контейнера
docker run --rm -it --name ardupilot_sitl -p 5760:5760 -p 14550:14550/udp ardupilot-sitl

## 6) Запустити ArduCopter SITL всередині контейнера
cd /ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --console --map \
  --out tcp:0.0.0.0:5760 \
  --out udp:0.0.0.0:14550

## 7) Перевірка конекту з Windows
Відкрийте новий Windows Terminal / CMD (контейнер має працювати), активуйте venv:
cd C:\Repos\DroneSim
.\.venv\Scripts\activate
python src\check.py
Очікувано:
OK heartbeat ...

## 8) Прогін одного сценарію (telemetry capture)
python src\run_one_connect.py --connect tcp:127.0.0.1:5760 --scenario scenarios\step_roll_10deg.json --params params\good.json --out-dir storage\run_001

## 9) Результат
C:\Repos\DroneSim\storage\run_001\telemetry.csv

## 10) Аналіз
python src\analyze_run.py --csv "C:\Repos\DroneSim\storage\run_001\telemetry.csv" --out "C:\Repos\DroneSim\storage\run_001\metrics.json"

## 11) Візуалізація
python src\visualize_run.py --csv "C:\Repos\DroneSim\storage\run_001\telemetry.csv" --metrics "C:\Repos\DroneSim\storage\run_001\metrics.json" --out-dir "C:\Repos\DroneSim\storage\run_001\plots"
