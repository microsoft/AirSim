setlocal
del /q gen_temp.txt
powershell -command "& { (Get-ItemProperty 'Registry::HKEY_CLASSES_ROOT\Unreal.ProjectFile\shell\rungenproj' -Name 'Icon' ).'Icon' } > gen_temp.tmp"
type gen_temp.tmp > gen_temp.txt
set /p gen_bin=<gen_temp.txt
del /q gen_temp.tmp
del /q gen_temp.txt
for %%f in (*.uproject) do (
		echo Generating files for %%f
		%gen_bin% /projectfiles "%cd%\%%f"
)
