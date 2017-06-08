msbuild NLoptNet.csproj /t:Build /p:Configuration="Release 4.5"
msbuild NLoptNet.csproj /t:Build;Package;Publish /p:Configuration="Release 4.0"