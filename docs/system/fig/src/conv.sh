#!/usr/bin/env bash
for f in build/*.pdf; do
  outf="${f%.pdf}.svg"
  echo in: $f, out: $outf
  inkscape -l "$outf" "$f"
  cp "$outf" ../
done
