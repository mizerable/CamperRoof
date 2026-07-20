$content = [System.IO.File]::ReadAllText("test/test_core_logic/test_main.cpp")
$content = $content -replace ', nullptr\)', ')'
$content = $content -replace ', bottomed_flags\)', ')'
$content = $content -replace ', false_flags\)', ')'
$content = $content -replace 'bool bottomed_flags\[4\].*?;\r?\n', ''
$content = $content -replace 'bool false_flags\[4\].*?;\r?\n', ''
[System.IO.File]::WriteAllText("test/test_core_logic/test_main.cpp", $content, [System.Text.Encoding]::UTF8)
