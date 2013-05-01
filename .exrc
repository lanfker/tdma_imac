if &cp | set nocp | endif
let s:cpo_save=&cpo
set cpo&vim
imap <silent> <expr> <F5> (pumvisible() ? "\<Plug>LookupFileCE" : "")."\\<Plug>LookupFile"
inoremap <Plug>LookupFileCE 
nmap  :BottomExplorerWindow
nmap  :FirstExplorerWindow
nmap <silent> ,fe :Sexplore!
map <silent> ,ee :e ~/.vimrc
map <silent> ,ss :source ~/.vimrc
nmap gx <Plug>NetrwBrowseX
nnoremap <silent> <Plug>NetrwBrowseX :call netrw#NetrwBrowseX(expand("<cWORD>"),0)
nmap <silent> <F5> <Plug>LookupFile
nmap <silent> <F9> :WMToggle
nmap <Nul>d :cs find d =expand("<cword>")
nmap <Nul>i :cs find i ^=expand("<cfile>")$
nmap <Nul>f :cs find f =expand("<cfile>")
nmap <Nul>e :cs find e =expand("<cword>")
nmap <Nul>t :cs find t =expand("<cword>")
nmap <Nul>c :cs find c =expand("<cword>")
nmap <Nul>g :cs find g =expand("<cword>")
nmap <Nul>s :cs find s =expand("<cword>")
inoremap <expr>  omni#cpp#maycomplete#Complete()
inoremap <expr> . omni#cpp#maycomplete#Dot()
inoremap <expr> : omni#cpp#maycomplete#Scope()
inoremap <expr> > omni#cpp#maycomplete#Arrow()
let &cpo=s:cpo_save
unlet s:cpo_save
set autoindent
set background=dark
set backspace=indent,eol,start
set balloonexpr=eclim#util#Balloon(eclim#util#GetLineError(line('.')))
set cindent
set completeopt=longest,menu
set cscopeprg=/usr/bin/cscope
set cscopetag
set cscopetagorder=1
set cscopeverbose
set expandtab
set fileencodings=ucs-bom,utf-8,default,latin1
set helplang=en
set hlsearch
set makeprg=./waf
set omnifunc=omni#cpp#complete#Main
set printoptions=paper:letter
set ruler
set runtimepath=~/.vim,/var/lib/vim/addons,/usr/share/vim/vimfiles,/usr/share/vim/vim73,/usr/share/vim/vimfiles/after,/var/lib/vim/addons/after,~/.vim/after,~/.vim/eclim,~/.vim/eclim/after
set shiftwidth=2
set smartindent
set smarttab
set suffixes=.bak,~,.swp,.o,.info,.aux,.log,.dvi,.bbl,.blg,.brf,.cb,.ind,.idx,.ilg,.inx,.out,.toc
set tabstop=2
set tags=~/workspace/iMac/NewTopologyTDMA/tags
" vim: set ft=vim :
