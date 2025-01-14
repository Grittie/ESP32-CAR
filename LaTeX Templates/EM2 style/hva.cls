\LoadClass[11pt]{article}
\renewcommand{\familydefault}{\sfdefault}
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{hva}[HBO\-ICT Report Format]

% Defines a new command for horizontal lines
\newcommand{\HRule}{\rule{0.8\linewidth}{0.5mm}}
 
% Defines the probability command
\newcommand{\Prob}[1]{\mathbf{Pr}\left\{ #1 \right\}}

\newcommand*\DefVar[1]{\@namedef{#1}##1{\global\@namedef{get#1}{##1}}}
\DefVar{course}

\RequirePackage{listings}
\RequirePackage{epsfig}
\RequirePackage{xcolor}

\usepackage[paper=a4paper, top=1in, bottom=1.75in, left=1in, right=1.25in]{geometry}

\RequirePackage[]{fancyhdr}
\pagestyle{fancy}
\fancyhead{}
\fancyfoot{}
\lhead{\includegraphics[scale=1]{img/hva.ps}}
\cfoot{\thepage}
\rfoot{\includegraphics[scale=0.8]{img/footer.ps}}

\newtheorem{question}{Question}[section]
\newtheorem{answer}{Answer}[section]

\lstdefinestyle{BashInputStyle}{
  language=[ANSI]C,
  basicstyle=\ttfamily\scriptsize,
  numbers=none,
  numberstyle=\tiny,
  numbersep=3pt,
  frame=tb,
  columns=fullflexible,
  backgroundcolor=\color{yellow!20},
  linewidth=0.90\linewidth,
  xleftmargin=0.05\linewidth
}

\def \maketitle {

  \begin{titlepage}

    \thispagestyle{empty}

    \begin{center}

      \begin{figure}[h!]
        \centerline{\psfig{file=img/hva.ps,scale=2.5}}
      \end{figure}

      \HRule
      \vspace{1cm}

      FACULTY OF DIGITAL MEDIA AND CREATIVE INDUSTRIES \\
      \textbf{HBO -- Information and Communication Technologies} \\

      \vspace{3cm} 

      {\Huge \textsc{\@title}}
      \vspace{1cm}

      {\large \textsc{\getcourse}}
      \vspace{2cm}

      \begin{center}
        \begin{tabular}{l}
          {\huge \@author}
        \end{tabular}
      \end{center}

      \vspace{6cm} 

      {\large \today}
      \vspace{1cm}

      \HRule

    \end{center}

  \end{titlepage}

}
