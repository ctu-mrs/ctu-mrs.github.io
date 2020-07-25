#!/usr/bin/env bash
for f in build/*.pdf; do
  outf="${f%.pdf}.png"
  echo in: $f, out: $outf
  convert ./"$f" ./"$outf"
  cp ./"${f%.pdf}.png" ../
done
