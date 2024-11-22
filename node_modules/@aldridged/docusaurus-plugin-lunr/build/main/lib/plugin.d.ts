import { LoadContext, Plugin } from '@docusaurus/types';
import { LoadedContent, PluginOptions } from './types';
export default function pluginContentLunr(context: LoadContext, opts: Partial<PluginOptions>): Plugin<LoadedContent | null>;
