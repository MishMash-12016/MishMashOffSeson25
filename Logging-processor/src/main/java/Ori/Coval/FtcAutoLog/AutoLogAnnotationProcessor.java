package Ori.Coval.FtcAutoLog;

import com.squareup.javapoet.ClassName;
import com.squareup.javapoet.JavaFile;
import com.squareup.javapoet.MethodSpec;
import com.squareup.javapoet.ParameterSpec;
import com.squareup.javapoet.TypeName;
import com.squareup.javapoet.TypeSpec;

import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.RoundEnvironment;
import javax.annotation.processing.SupportedAnnotationTypes;
import javax.annotation.processing.SupportedSourceVersion;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.PackageElement;
import javax.lang.model.element.TypeElement;
import javax.lang.model.element.VariableElement;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.type.TypeKind;
import javax.lang.model.type.TypeMirror;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Annotation processor that generates an AutoLogged subclass which
 * overrides fields and methods to log via WpiLog.
 *
 * @author ori coval
 */
@SupportedAnnotationTypes({
        "Ori.Coval.Logging.AutoLog",
        "Ori.Coval.Logging.AutoLogAndPostToFtcDashboard"
})
@SupportedSourceVersion(SourceVersion.RELEASE_8)
public class AutoLogAnnotationProcessor extends AbstractProcessor {
    private static final ClassName WPILOG = ClassName.get("Ori.Coval.Logging", "WpiLog");
    private static final ClassName FTC_DASHBOARD = ClassName.get("com.acmerobotics.dashboard", "FtcDashboard");
    private static final ClassName LOGGED = ClassName.get("Ori.Coval.Logging", "Logged");
    private static final ClassName AUTO_LOG_MANAGER = ClassName.get("Ori.Coval.Logging", "AutoLogManager");
    private static final ClassName SUPPLIER_LOG = ClassName.get("Ori.Coval.Logging", "SupplierLog");

    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        for (TypeElement annotation : annotations) {
            for (Element e : roundEnv.getElementsAnnotatedWith(annotation)) {
                if (e.getKind() == ElementKind.CLASS) {
                    boolean postToFtc = annotation.getQualifiedName()
                            .toString()
                            .equals("Ori.Coval.Logging.AutoLogAndPostToFtcDashboard");
                    generate((TypeElement) e, postToFtc);
                }
            }
        }
        return true;
    }

    private void generate(TypeElement classElem, boolean postToFtcDashboard) {
        String pkg = getPackageName(classElem);
        String orig = classElem.getSimpleName().toString();
        String autoName = orig + "AutoLogged";
        ClassName rawClass = ClassName.get(pkg, orig);

        // Build subclass
        TypeSpec.Builder clsBuilder = TypeSpec.classBuilder(autoName)
                .addModifiers(Modifier.PUBLIC)
                .addSuperinterface(LOGGED)
                .superclass(TypeName.get(classElem.asType()));

        // toLog() method builder
        MethodSpec.Builder toLog = MethodSpec.methodBuilder("toLog")
                .addModifiers(Modifier.PUBLIC)
                .addJavadoc("Auto-generated telemetry logging\n");

        // Collect supplier fields
        List<String> supplierFields = new ArrayList<>();
        List<String> supplierKeys = new ArrayList<>();

        // Process fields (include static, skip private)
        for (Element fe : classElem.getEnclosedElements()) {
            if (fe.getKind() != ElementKind.FIELD) continue;
            VariableElement field = (VariableElement) fe;
            Set<Modifier> mods = field.getModifiers();
            if (mods.contains(Modifier.PRIVATE)) continue;

            String fname = field.getSimpleName().toString();
            TypeMirror t = field.asType();
            TypeKind k = t.getKind();

            boolean isSupplier = (k == TypeKind.DECLARED &&
                    (t.toString().equals("java.util.function.LongSupplier") ||
                            t.toString().equals("java.util.function.DoubleSupplier") ||
                            t.toString().equals("java.util.function.IntSupplier") ||
                            t.toString().equals("java.util.function.BooleanSupplier")));

            if (!(k.isPrimitive() ||
                    (k == TypeKind.DECLARED && t.toString().equals("java.lang.String")) ||
                    k == TypeKind.ARRAY ||
                    isSupplier)) continue;

            String key = orig + "." + fname;
            String fieldRef;
            if (mods.contains(Modifier.STATIC)) {
                // static field: reference via ClassName.FIELD
                fieldRef = rawClass.simpleName() + "." + fname;
            } else {
                // instance field: this.FIELD
                fieldRef = "this." + fname;
            }

            if (isSupplier) {
                supplierFields.add(fname);
                supplierKeys.add(key);
            } else {
                toLog.addStatement(
                        "$T.getInstance().log($S, $L)",
                        WPILOG, key, fieldRef
                );
                if (postToFtcDashboard) {
                    toLog.addStatement(
                            "$T.getInstance().getTelemetry().addData($S, $L)",
                            FTC_DASHBOARD, key, fieldRef
                    );
                }
            }
        }

        // Generate constructors for each original constructor
        for (Element enclosed : classElem.getEnclosedElements()) {
            if (enclosed.getKind() != ElementKind.CONSTRUCTOR) continue;
            ExecutableElement ctorElem = (ExecutableElement) enclosed;

            List<ParameterSpec> params = new ArrayList<>();
            List<String> paramNames = new ArrayList<>();
            for (VariableElement p : ctorElem.getParameters()) {
                String nm = p.getSimpleName().toString();
                params.add(ParameterSpec.builder(TypeName.get(p.asType()), nm).build());
                paramNames.add(nm);
            }

            MethodSpec.Builder ctor = MethodSpec.constructorBuilder()
                    .addModifiers(Modifier.PUBLIC)
                    .addParameters(params)
                    .addStatement("super($L)", String.join(", ", paramNames));

            // Wrap any supplier fields
            if (!supplierFields.isEmpty()) {
                for (int i = 0; i < supplierFields.size(); i++) {
                    String fname = supplierFields.get(i);
                    String key = supplierKeys.get(i);
                    ctor.addStatement(
                            "super.$L = $T.wrap($S, super.$L, $L)",
                            fname, SUPPLIER_LOG, key, fname, postToFtcDashboard
                    );
                }
            }

            ctor.addStatement("$T.register(this)", AUTO_LOG_MANAGER);
            clsBuilder.addMethod(ctor.build());
        }

        // Override methods
        for (Element me : classElem.getEnclosedElements()) {
            if (me.getKind() != ElementKind.METHOD) continue;
            ExecutableElement method = (ExecutableElement) me;
            Set<Modifier> mmods = method.getModifiers();
            if (!mmods.contains(Modifier.PUBLIC) || mmods.contains(Modifier.STATIC)) continue;

            TypeMirror rt = method.getReturnType();
            TypeKind rtk = rt.getKind();
            if (!(rtk.isPrimitive() || (rtk == TypeKind.DECLARED && rt.toString().equals("java.lang.String"))))
                continue;

            String mname = method.getSimpleName().toString();
            TypeName rtn = TypeName.get(rt);
            String key = orig + "." + mname;

            // Build parameter list and invocation string
            StringBuilder argList = new StringBuilder();
            List<ParameterSpec> paramSpecs = new ArrayList<>();
            for (VariableElement p : method.getParameters()) {
                String nm = p.getSimpleName().toString();
                paramSpecs.add(ParameterSpec.builder(TypeName.get(p.asType()), nm).build());
                if (argList.length() > 0) argList.append(", ");
                argList.append(nm);
            }

            MethodSpec.Builder override = MethodSpec.methodBuilder(mname)
                    .addAnnotation(Override.class)
                    .addModifiers(Modifier.PUBLIC)
                    .returns(rtn)
                    .addParameters(paramSpecs)
                    .addStatement("$T result = super.$L($L)", rtn, mname, argList.toString());

            if (postToFtcDashboard) {
                override.addStatement(
                        "$T.getInstance().getTelemetry().addData($S, result)",
                        FTC_DASHBOARD, key
                );
            }

            override.addStatement(
                    "return $T.getInstance().log($S, result)",
                    WPILOG, key
            );

            clsBuilder.addMethod(override.build());
        }

        // Add the toLog() method
        clsBuilder.addMethod(toLog.build());

        // Write out the file
        try {
            JavaFile.builder(pkg, clsBuilder.build())
                    .build()
                    .writeTo(processingEnv.getFiler());
        } catch (IOException ex) {
            processingEnv.getMessager().printMessage(
                    javax.tools.Diagnostic.Kind.ERROR,
                    "Failed to write AutoLogged: " + ex.getMessage()
            );
        }
    }

    private String getPackageName(TypeElement t) {
        Element e = t;
        while (e != null && !(e instanceof PackageElement)) {
            e = e.getEnclosingElement();
        }
        return e == null
                ? null
                : ((PackageElement) e).getQualifiedName().toString();
    }
}
