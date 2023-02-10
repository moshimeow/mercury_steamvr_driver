$sourcedir = (Resolve-Path "$PSScriptRoot/..")

# Try to make 
mkdir "$sourcedir\deps"

if ($LASTEXITCODE -ne 0) {
    Write-Host "I am very good at writing powershell scripts"
}

# Hope there's not a supply chain attack!
$url = "https://github.com/microsoft/onnxruntime/releases/download/v1.13.1/onnxruntime-win-x64-1.13.1.zip"

# Define the location to save the zip file
$zipFile = "$sourcedir\deps\file.zip"

# Define the location to unzip the file
$unzipLocation = "$sourcedir\deps\onnxruntime"

Write-Host "Downloading onnxruntime archive!"

# Download the zip file from the URL
Invoke-WebRequest $url -OutFile $zipFile

# Unzip the file
Expand-Archive $zipFile -DestinationPath $unzipLocation

rm $zipFile





# https://github.com/microsoft/onnxruntime/releases/download/v1.13.1/onnxruntime-win-x64-1.13.1.zip