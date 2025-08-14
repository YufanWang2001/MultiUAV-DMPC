# latexmk configuration for XeLaTeX + biber
$pdflatex = 'xelatex -interaction=nonstopmode -shell-escape -file-line-error %O %S';
$biber = 'biber %O %S';
$bibtex = 'bibtex %O %S';
$use_biber = 1;   # set to 0 if using bibtex
$clean_ext = "acn acr alg aux bbl bcf blg dvi fdb_latexmk fls glg glo gls idx ilg ind ist lof log lot nav out run.xml snm synctex.gz toc vrb xdv";
