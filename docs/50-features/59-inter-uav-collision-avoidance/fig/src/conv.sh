#!/usr/bin/env bash

for t in *.tex; do
  latex -output-directory=build "$t" > /dev/null
  if [ $? -ne 0 ]; then
    echo "Warning: the command 'latex -output-directory=build $t' failed!"
  fi
done

for f in build/*.dvi; do
  outf="${f%.dvi}.svg"
  echo in: $f, out: $outf
  dvisvgm --no-fonts "$f" -o "$outf" > /dev/null
  if [ $? -ne 0 ]; then
    echo "Warning: the command 'dvisvgm --no-fonts $f $outf' failed!"
  fi
  mv "$outf" ../
done
