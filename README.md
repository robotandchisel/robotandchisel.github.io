# Robot & Chisel

This is my new website, merging several previous ones:

 * importing my blog (showusyoursensors.com) from blogger
 * replacing mostly static front page at fer.gs

The template (hyde) is open sourced under the [MIT license](LICENSE.md).

## Deploy Notes

Setting up on Mac:
```
gem install --user-install bundler jekyll
bundler  (will install dependencies)
```

Building locally:
```
export GEM_HOME=$HOME/.gem/
bundler exec jekyll serve
```

To serve un-published future posts:
```
bundler exec jekyll serve --future
```
