# GCE 稿件投稿前终检

检查日期：2026-09-11。检查者：Codex。

结论：本地终检通过。分页与两张表的标题上边距已修复，未发现剩余的稿件制品阻断项。正式上传、在线 PDF 检验和提交尚未执行。

PDF SHA-256：`d009d41eee1e05d9a50b99486c5954e8dae2763daf75b40a6a7addebff4356da`

## 已检查

- 8 页：7 页完整正文，第 8 页仅致谢与参考文献；US Letter，官方双栏模板，类文件与参考文献样式未修改。
- 全部 8 页逐页查看。4 幅正文矢量图、4 张表和 31 条参考文献完整可读，无越栏、遮挡、缺字或未解析引用；两张页首表题加入 5 pt 正向间距后通过渲染边距检查。
- 23 个字体全部嵌入，无 Type 3、PDF 注释、超链接、书签、附件或加密。作者栏及作者元数据为空；正文没有作者账户、邮箱或本地路径，AI 使用披露保留在致谢中。
- 31 条正文引文均匹配已核实书目记录，27 个非 arXiv DOI 完整显示。此次复核引用映射与现有来源记录，没有将所有原论文重新通读。
- 公式与实现对照覆盖支持集合、历史权重、正负准入、Bernoulli 归一化、高斯积分、曲率保护和包长公式，未发现定义不一致。
- 43 段 V2V 的 OSPA 降幅 5.8%/4.4% 以 No-age KLA 为对照；原 25 段的分片字节节省 11.3%/10.8% 以完整 GCE 包为对照。全文分母与比较对象一致。
- 本轮重新从保存的来源汇总生成图表与数值，并复算 854 行结果、180 项配对比较。71 个受保护制品中仅两份表容器增加间距，表内数据、数值宏、图形源、参考文献与主模板内容均保持一致。
- 解压源码包，在独立临时目录重新生成图表并构建；228 个输入文件、52 个再生文件匹配，8 页文字和渲染像素完全一致。

## 论文收束

GCE 是唯一主方法，摘要、引言贡献和结论围绕选择性准入当前信息、联合更新空间与存在概率、以及精确零向量编码展开。重复的局限表述已压缩；支撑比较所需的数据范围和实际结果保留。延迟投影增强与联合关联扩展未进入论文。

## 投稿操作

使用 `output/pdf/icra2027_draft.pdf` 投稿；当前为 PDF 1.5，大小 2,037,605 字节（1.94 MiB）。

在 [PaperPlaza PDF Test](https://ras.papercept.net/conferences/scripts/pdftest.pl) 完成会议专用的在线检验，并在投稿系统核对全部作者、单位及关键词。当前检查未上传论文，未验证动态检查器中的文件大小限制，也未代替作者确认研究内容。

依据 [ICRA 2027 官方说明](https://2027.ieee-icra.org/contribute/call-for-icra-2027-papers-now-accepting-submissions/)，八页包含正文、图表、致谢及参考文献；所有作者须在首次提交时录入系统，PDF 保持匿名。源码 ZIP 用于归档和复现，会议额外附件仅允许视频。

版式和匿名对照来源：[RAS 双匿名规则](https://www.ieee-ras.org/publications/rules-for-the-double-anonymous-review-process/)、[PaperPlaza PDF 要求](https://ras.papercept.net/conferences/support/general.php)、[页面设置](https://ras.papercept.net/conferences/support/page.php)。

相关记录：`artifact_qa.json`、`visual_review.md`、`portable_rebuild.json`、`final_submission_check.json`。

复核范围为稿件与制品、保存结果的重算和同一运行环境下的可移植构建；本轮没有重跑检测器或完整跟踪实验。
