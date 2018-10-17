PAUSE Make sure to increment version in setup.py. Continue?
python setup.py sdist
twine upload --repository-url https://upload.pypi.org/legacy/ dist/*
REM pip install airsim --upgrade
REM pip show airsim

REM pip install yolk3k
REM yolk -V airsim