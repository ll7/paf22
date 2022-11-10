# Title of wiki page

**Summary:** This page functions a template for who to build knowledge articles for everyone to understand. The basic structure should be kept for all articles. This template further contains a cheat sheet with the most useful markdown syntax.


---
### Author

Josef Kircher

### Date

04.11.2022

### Prerequisite
VSCode Extensions:

- Markdown All in One by Yu Zhang (for TOC)

---

How to generate a TOC in VSCode and Pycharm:

VSCode: 

1. Install Markdown All in One via Extensions
2. ``Ctrl+Shift+P``
3. Command "Create Table of Contents"

Cosmetic change: Markdown All in One uses `-` as unordered list indicator, to change it to `*` like in Pycharm 

Go to Extension setting and change it under `Markdown>Extension>Toc>Unordered List:Marker`

Pycharm:

1. ``Alt+Ins``
2. Select Table of Contents
3. To update Table of Contents follow Step 1. and select Update Table of Contents

* [Title of wiki page](#title-of-wiki-page)
    * [Author](#author)
    * [Date](#date)
    * [Prerequisite](#prerequisite)
* [Cheat Sheet](#cheat-sheet)
  * [Basics](#basics)
* [H1](#h1)
  * [H2](#h2)
  * [### H3](#-h3)
  * [> blockquote](#-blockquote)
  * [Extended](#extended)
    * [My Great Heading {#custom-id}](#my-great-heading-custom-id)
* [more Content](#more-content)
    * [Sources](#sources)

# Cheat Sheet

## Basics

---

Headings:
# H1
## H2
### H3
---
Bold 

**bold text**

---
Italic 

*italicized text*

---
Blockquote

> blockquote
---
Ordered List
1. First item
2. Second item
3. Third item

---
Unordered List
- First item
- Second item
- Third item

---
Code 	

`code`

---

Horizontal Rule

---

Link
[title](https://www.example.com)

---
Image
![alt text](image.jpg)

## Extended

---
Table
| Syntax | Description |
| ----------- | ----------- |
| Header | Title |
| Paragraph | Text |

---
Fenced Code Block
```
{
  "firstName": "John",
  "lastName": "Smith",
  "age": 25
}
```

---
Footnote 	

Here's a sentence with a footnote. [^1]

[^1]: This is the footnote.

---
Heading ID 	

### My Great Heading {#custom-id}

---
Definition List 
term
: definition

---
Strikethrough 	

~~The world is flat.~~

---

Task List
- [x] Write the press release
- [ ] Update the website
- [ ] Contact the media

---

Subscript 	

H~2~O

---

Superscript 	

X^2^ 

---

# more Content


### Sources
https://www.markdownguide.org/cheat-sheet/