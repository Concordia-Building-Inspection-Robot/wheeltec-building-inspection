#!/usr/bin/env bash

python -m venv venv
source venv/bin/activate

pip install -r requirements.txt
pip install -r UI/requirements.txt

python setup.py develop
