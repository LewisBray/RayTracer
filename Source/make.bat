@ECHO OFF
@SETLOCAL

SET start=%time%
clang main.cpp -o ray_tracer.exe -std=c++1z -g -mavx2 -mfma --for-linker=/INCREMENTAL:NO
SET end=%time%

FOR /f "tokens=1-4 delims=:." %%a in ("%start%") DO (
    SET /a start_s=%%c
    SET /a start_cs=%%d
)

FOR /f "tokens=1-4 delims=:." %%a in ("%end%") DO (
    SET /a end_s=%%c
    SET /a end_cs=%%d
)

SET /a centisecs=%end_cs%-%start_cs%
SET /a secs=%end_s%-%start_s%
IF %centisecs% LSS 0 (
    SET /a secs=%secs%-1
    SET /a centisecs=100%centisecs%
)

IF %secs% LSS 0 (
    SET /a secs=60%secs%
)

IF 1%centisecs% LSS 100 (
    SET /a centisecs=0%centisecs%
)

ECHO Build time: %secs%.%centisecs%s
