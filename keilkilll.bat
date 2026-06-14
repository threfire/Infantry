::del *.bak /s  ::备份文件，可能包含非编译产物
::del *.ddk /s  ::工具/配置类文件，保留
::del *.edk /s  ::工具/配置类文件，保留
del *.lst /s
del *.lnp /s
::del *.mpf /s  ::工程/工具类文件，保留
::del *.mpj /s  ::工程/工具类文件，保留
del *.obj /s
del *.omf /s
::del *.opt /s  ::不允许删除JLINK的设置
del *.plg /s
del *.rpt /s
::del *.tmp /s  ::通用临时文件，可能包含非编译产物
del *.__i /s
del *.crf /s
del *.o /s
del *.d /s
del *.axf /s
del *.tra /s
del *.dep /s           
::del JLinkLog.txt /s  ::JLink日志，保留

del *.iex /s
::del *.htm /s  ::网页/文档类文件，可能包含非编译产物
del *.map /s
exit
