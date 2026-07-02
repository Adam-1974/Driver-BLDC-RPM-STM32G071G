# Driver BLDC RPM (STM32G071G)

Sterownik BLDC dla STM32G071GBU6.

Pierwszy etap repozytorium zawiera:

- strukture projektu pod STM32CubeIDE 2.0.0,
- konfiguracje zalozen hardware i pinow,
- globalna strukture NVM z parametrami regulatora,
- szkic regulatora trybow SINUS / 6-step,
- modul odczytu BEMF oparty na implementacji AM32 dla STM32G071.

## Cel

Regulator ma pracowac w dwoch trybach:

- niski zakres obrotow: modulacja sinusoidalna z tabela sinus,
- wyzszy zakres obrotow: 6-step z komutacja z BEMF.

Predkosc zadawana jest w RPM z rozdzielczoscia 1 RPM. Odczyt predkosci z BEMF jest prowadzony jako eRPM. Kierunek pracy: lewo i prawo.

## Najwazniejsze pliki

- `Driver_BLDC_RPM_STM32G071G.ioc` - punkt startowy konfiguracji CubeMX/CubeIDE.
- `Core/Inc/board.h` - mapa pinow projektu.
- `Core/Inc/nvm_config.h` - jedna globalna struktura parametrow NVM.
- `Core/Src/bemf_am32.c` - odczyt BEMF przeniesiony/adaptowany z AM32 dla STM32G071.
- `docs/project_assumptions.md` - zalozenia projektu i decyzje.
- `docs/hardware.md` - konfiguracja peryferiow i uwagi do pinow.
- `docs/am32_bemf_notes.md` - notatki o integracji z AM32.

## AM32

Projekt korzysta z AM32 jako zrodla referencyjnego, bez kopiowania calego firmware.
Fragmenty BEMF sa oznaczone w kodzie i pochodza z:

https://github.com/am32-firmware/AM32

AM32 jest licencjonowany jako GPL-3.0. Ten projekt przyjmuje licencje GPL-3.0-or-later dla plikow z kodem C.

