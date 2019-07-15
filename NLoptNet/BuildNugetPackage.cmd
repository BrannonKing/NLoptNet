msbuild NLoptNet.csproj /t:Build /p:Configuration="Release 4.5" /p:Platform="AnyCPU"
msbuild NLoptNet.csproj /t:Build;Package;Publish /p:Configuration="Release 4.0" /p:Platform="AnyCPU"