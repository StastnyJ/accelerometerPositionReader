
.PHONY: all Report.pdf clean
all: Report.pdf
Report.pdf:
	TEXINPUTS=:. latexmk -pdf -silent Report.tex
clean:
	rm -f *.aux *.bbl *.blg *.fls *.fdb_latexmk *.log *.pdf *~ Report.out
