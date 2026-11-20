@echo off
cd /d "%~dp0"
title CP3 agent runner
.venv\Scripts\python.exe agent_runner.py
pause
