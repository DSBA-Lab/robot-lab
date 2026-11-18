#!/bin/sh
# floorplan.html(본문 조각)을 감싸 GitHub Pages용 index.html을 만듭니다. floorplan.html을 고친 뒤 실행하세요.
cd "$(dirname "$0")/.." || exit 1
{
  printf '%s\n' '<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><link rel="icon" type="image/png" sizes="64x64" href="assets/favicon.png"><link rel="icon" type="image/svg+xml" href="assets/favicon.svg"><style>body{margin:0;font:14px system-ui,sans-serif}img{max-width:100%}[hidden]{display:none!important}</style></head><body>'
  cat floorplan.html
  printf '%s\n' '</body></html>'
} > index.html
