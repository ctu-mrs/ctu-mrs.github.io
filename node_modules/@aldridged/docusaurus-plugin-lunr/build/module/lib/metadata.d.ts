import { MetadataRaw } from './types';
export default function processMetadata({ source, refDir, context, options, env, }: {
    source: any;
    refDir: any;
    context: any;
    options: any;
    env: any;
}): Promise<MetadataRaw>;
