# This page is dedicated to simple hints for Latex beginners
## Footnote in figure caption
I had an issue when I had a figure, which had a footnote in its caption, but the problems with captions and footnote that it won't be displayed as a normal footnote. So there is a workaround that:
``` 

\begin{figure}
   \centering
   \includegraphics{foo}  ...
   \caption[Caption for LOF]{Real caption\protect\footnotemark}
\end{figure}

\footnotetext{blah blah blah}
 ```
You simple add the caption and add two tags "protect" and "footnotemark" and then manually add the footnote text by "footnotetext"